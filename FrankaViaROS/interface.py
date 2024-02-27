import threading
from abc import ABC,abstractmethod

import os
import torch
import torchcontrol as toco
from torchcontrol.transform import Transformation as T
from torchcontrol.transform import Rotation as R

from typing import Tuple,List
import numpy as np
import transformations as tf
import copy
import time
from utily import to_PoseStamped

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

from FrankaViaROS.franka_robot import *

POLYMETIS_ROOT = '/home/wenhai/pub_repo/fairo/polymetis/polymetis'
torch.classes.load_library(os.path.join(POLYMETIS_ROOT,'build/torch_isolation/libtorchscript_pinocchio.so'))

URDF_PATH = os.path.join(POLYMETIS_ROOT,'data/franka_panda/panda_arm.urdf')


class BaseRobotInterface(ABC):
    def __init__(self,urdf_path=URDF_PATH,time_to_go_default: float = 1.0):
        # self.robot_model = torch.classes.torchscript_pinocchio.RobotModelPinocchio(urdf_path,False)
        self.robot_model = toco.models.RobotModelPinocchio(urdf_path,'panda_link8')
        self.time_to_go_default = time_to_go_default
        self.rest_joint = None
        self.set_home_pose(torch.Tensor([-0.1393, -0.0204, -0.0520, -2.06912, 0.0505, 2.0028, -0.91678]))

    @abstractmethod
    def get_robot_state(self):
        raise NotImplementedError

    def get_joint_positions(self):
        return torch.Tensor(self.get_robot_state().q)

    def get_joint_velocities(self):
        return torch.Tensor(self.get_robot_state().q_d)

    def get_ee_pose(self):
        return torch.Tensor(self.get_robot_state().O_T_EE).resize(4,4).transpose(1,0)

    def set_home_pose(self, home_pose: torch.Tensor):
        """Sets the home pose for `go_home()` to use."""
        self.rest_joint = home_pose

    def get_rest_joint(self):
        return self.rest_joint



    def _adaptive_time_to_go(self, joint_displacement: torch.Tensor):
        """Compute adaptive time_to_go
        Computes the corresponding time_to_go such that the mean velocity is equal to one-eighth
        of the joint velocity limit:
        time_to_go = max_i(joint_displacement[i] / (joint_velocity_limit[i] / 8))
        (Note 1: The magic number 8 is deemed reasonable from hardware tests on a Franka Emika.)
        (Note 2: In a min-jerk trajectory, maximum velocity is equal to 1.875 * mean velocity.)

        The resulting time_to_go is also clipped to a minimum value of the default time_to_go.
        """
        joint_vel_limits = self.robot_model.get_joint_velocity_limits()
        joint_pos_diff = torch.abs(joint_displacement)
        time_to_go = torch.max(joint_pos_diff / joint_vel_limits * 8.0)
        return max(time_to_go, self.time_to_go_default)

    def solve_inverse_kinematics(
        self,
        position: torch.Tensor,
        orientation: torch.Tensor,
        q0: torch.Tensor,
        tol: float = 1e-3,
    ) -> Tuple[torch.Tensor, bool]:
        """Compute inverse kinematics given desired EE pose"""
        # Call IK
        joint_pos_output = self.robot_model.inverse_kinematics(
            position, orientation, rest_pose=q0
        )

        # Check result
        pos_output, quat_output = self.robot_model.forward_kinematics(joint_pos_output)
        pose_desired = T.from_rot_xyz(R.from_quat(orientation), position)
        pose_output = T.from_rot_xyz(R.from_quat(quat_output), pos_output)
        err = torch.linalg.norm((pose_desired * pose_output.inv()).as_twist())
        ik_sol_found = err < tol

        return joint_pos_output, ik_sol_found

class FrankaInterface(BaseRobotInterface):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.robot_connection = FrankaMoveViaRos()
        self.robot_connection.switch_to_mode("effort_trajectory")
        # self.robot_connection.switch_to_mode("gravity_compensation")
        # import pdb;pdb.set_trace()
        self.trajectory = []
        self.record_trajectory_threading = threading.Thread(target=self.record_trajectory,daemon=True)
        self.record = True

        self.real_ee_marker_pub = rospy.Publisher('/real_ee_marker',MarkerArray,queue_size = 500)

    def pub_marker(self,pub, pos):
        marker = Marker()

        marker.header.frame_id = "panda_link0"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0

        # Set the scale of the marker
        scale = 0.01
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        pub.publish(marker)

    def pub_markerarray(self):
        markers = MarkerArray()
        for i, tip in enumerate(self.trajectory):
            marker = Marker()

            marker.header.frame_id = "panda_link0"
            # marker.header.stamp = rospy.Time.now()

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = 2
            marker.id = i
            marker.action = 0

            # Set the scale of the marker
            scale = 0.005
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale

            # Set the color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Set the pose of the marker
            marker.pose.position.x = tip[0]
            marker.pose.position.y = tip[1]
            marker.pose.position.z = tip[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            markers.markers.append(marker)
        self.real_ee_marker_pub.publish(markers)

    def record_trajectory(self, delta_time=0.5):
        while True:
            time.sleep(delta_time)
            tip_pos = self.get_ee_pose().numpy()[:3, 3]
            self.trajectory.append(tip_pos.tolist())
            # self.pub_marker(self.real_ee_marker_pub, tip_pos)
            self.pub_markerarray()
            if not self.record:
                self.record = True
                break

    def get_robot_state(self):
        return self.robot_connection.get_robotstates()

    def get_robot_ee_pose(self):
        return self.robot_connection

    def open_gripper(self):
        self.robot_connection.gripper.open()
    def close_gripper(self):
        self.robot_connection.gripper.grasp()

    def move_to_joint_positions(self,
                                positions: torch.Tensor,
                                time_to_go: float = None,
                                delta: bool = False,
                                block_time: float = 100,
                                ):
        # Parse parameters
        joint_pos_current = self.get_joint_positions()
        joint_pos_desired = torch.Tensor(positions)
        if delta:
            joint_pos_desired += joint_pos_current

        time_to_go_adaptive = self._adaptive_time_to_go(
            joint_pos_desired - joint_pos_current
        )
        if time_to_go is None:
            time_to_go = time_to_go_adaptive
        elif time_to_go < time_to_go_adaptive:
            print(
                "The specified 'time_to_go' might not be large enough to ensure accurate movement."
            )
            time_to_go = time_to_go_adaptive

        self.robot_connection.move_to_joint(joint_pos_desired.numpy().tolist(),time_to_go, block_time=block_time)
        return time_to_go

    def pub_axis(self,planed_time):
        # publish some hard-coded estimated axis
        estimated_axis_publisher1 = rospy.Publisher("estimated/axis1", PoseStamped, queue_size=1)
        estimated_axis_publisher2 = rospy.Publisher("estimated/axis2", PoseStamped, queue_size=1)
        estimated_axis_publisher3 = rospy.Publisher("estimated/axis3", PoseStamped, queue_size=1)
        time.sleep(planed_time/3.)
        joint1 = np.array([0.7874264, 0.19766188, 0.12789592, 0.06077503, 0.01576262, 0.93844676])
        joint2 = np.array([0.7674264, 0.15766188, 0.10789592, 0.08077503, 0.01676262, 0.90844676])
        joint3 = np.array([0.7274264, 0.13766188, 0.0789592, 0.01077503, 0.01676262, 0.95844676])
        estimated_axis_publisher1.publish(to_PoseStamped(joint1, stamp=rospy.Time.now(), frame_id='panda_link0'))
        time.sleep(planed_time / 3.)
        estimated_axis_publisher2.publish(to_PoseStamped(joint2, stamp=rospy.Time.now(), frame_id='panda_link0'))
        time.sleep(planed_time / 3.)
        estimated_axis_publisher3.publish(to_PoseStamped(joint3, stamp=rospy.Time.now(), frame_id='panda_link0'))

    def execute_articulated_trajectory(self,axis_origin,axis_direction,block_time=500):
        robot_state = self.get_robot_state()
        joint_current = [np.array(robot_state.q), np.array(robot_state.dq), np.array([0.]*7)]
        grasp_pose = self.get_ee_pose()
        waypoints, planed_angle, planed_time, planed_num = self.get_articulated_trajectory(joint_current,grasp_pose,axis_origin,axis_direction)
        print('planed_angle: ',np.degrees(planed_angle))
        print('planed_time: ',planed_time)
        print('planed_num: ',planed_num)
        print('waypoints num: ',len(waypoints))
        threading.Thread(target=self.pub_axis,args=(planed_time,),daemon=True).start()

        self.record_trajectory_threading.start()
        self.robot_connection.follow_joint_trajectory(waypoints, block_time)
        self.record = False
        return planed_time, planed_num

    def execute_straight_trajectory(self,axis_direction,block_time):
        robot_state = self.get_robot_state()
        ee_pose = self.get_ee_pose()
        joint_current = [np.array(robot_state.q), np.array(robot_state.dq), np.array([0.] * 7)]
        waypoints, planed_dis, planed_time, planed_num = self.get_staight_trajectory(joint_current, ee_pose, axis_direction)
        # print('planed_distance: ', np.degrees(planed_dis))
        # print('planed_time: ', planed_time)
        # print('planed_num: ', planed_num)
        # print('waypoints num: ', len(waypoints))
        self.record_trajectory_threading.start()
        self.robot_connection.follow_joint_trajectory(waypoints, block_time)
        self.record = False
        return planed_time, planed_num


    def execute_online_articulated_trajectory(self,axis_origin,axis_direction,block_time=500):
        robot_state = self.get_robot_state()
        joint_current = [np.array(robot_state.q), np.array(robot_state.dq), np.array([0.]*7)]
        grasp_pose = self.get_ee_pose()
        waypoints, planed_angle, planed_time, planed_num = self.get_articulated_trajectory(joint_current,grasp_pose,axis_origin,axis_direction)

        print('planed_angle: ',np.degrees(planed_angle))
        print('planed_time: ',planed_time)
        print('planed_num: ',planed_num)
        print('waypoints num: ',len(waypoints))
        import pdb;pdb.set_trace()
        seg_num = 3
        pre_joint_pose = np.concatenate([axis_origin, axis_direction])
        for i in range(seg_num):
            waypoints_1 = waypoints[:int(planed_num / (seg_num - float(i)))]
            self.follow_joint_trajectory(waypoints_1, block_time)

            joint_pose = self.fine_tune_revolute_joint_pose(pre_joint_pose,
                                                            np.array(self.trajectory).astype(np.float32),
                                                            max_iter=10)
            estimated_axis_publisher1 = rospy.Publisher("estimated/axis%d"%i, PoseStamped, queue_size=1)
            estimated_axis_publisher1.publish(
                to_PoseStamped(joint_pose[0], stamp=rospy.Time.now(), frame_id='panda_link0'))
            pre_joint_pose = copy.deepcopy(joint_pose[0])
            axis_origin, axis_direction = pre_joint_pose[:3], pre_joint_pose[3:]
            import pdb;pdb.set_trace()
            robot_state = self.get_robot_state()
            joint_current = [np.array(robot_state.q), np.array(robot_state.dq), np.array([0.] * 7)]
            grasp_pose = self.get_ee_pose()
            waypoints, planed_angle, planed_time, planed_num = self.get_articulated_trajectory(joint_current,
                                                                                               grasp_pose, axis_origin,
                                                                                               axis_direction)
        # self.robot_connection.follow_joint_trajectory((waypoints_1,block_time))
        # self.record_trajectory_threading.start()
        # self.robot_connection.follow_joint_trajectory(waypoints, block_time)
        # self.record = False

        return planed_time, planed_num

    def follow_joint_trajectory(self,waypoints, block_time):
        self.record_trajectory_threading.start()
        self.robot_connection.follow_joint_trajectory(waypoints, block_time)
        self.record = False

    def check_joint_limits(self,joints,threshold=0.2):
        joint_limits = self.robot_model.get_joint_angle_limits()
        lower_limits = joint_limits[0]
        uper_limits = joint_limits[1]
        in_limit = True
        for i, joint in enumerate(joints):
            if joint <= lower_limits[i] + threshold or joint >= uper_limits[i] - threshold:
                in_limit = False
        return in_limit

    def get_online_extimated_axis(self,real_trajectory, axis_origin, axis_direction,max_iter=10):
        axis_direction = axis_direction / torch.norm(axis_direction)
        joint_pose = torch.concatenate([axis_origin,axis_direction])

        real_trajectory = torch.from_numpy(real_trajectory).to(axis_origin.dtype)
        mean_traj = real_trajectory.mean(axis=0)
        distances = torch.norm(real_trajectory - mean_traj,dim=1)
        traj_center = real_trajectory[torch.argmin(distances)]

        joint_pose.requires_grad = True
        optimizer = torch.optim.Adam([joint_pose],lr=0.0005,betas=(0.9, 0.999))
        scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=10, gamma=0.95)
        loss_f = torch.nn.L1Loss()
        min_loss = 10000
        min_step = 0
        best_joint_pose = copy.deepcopy(joint_pose)

        while True:
            pre_trajectory = self.axis_to_trajectory(joint_pose[:3],joint_pose[3:],traj_center)
            # pre_trajectory = torch.from_numpy(pre_trajectory) # n * 3

            dis_matrix = torch.norm(real_trajectory[:,None,:] - pre_trajectory[None,:,:], dim=2) # N * 1 * 3 - 1 * 1000 * 3
            index = torch.argmin(dis_matrix, dim=1)
            direction_loss = loss_f(pre_trajectory[index], real_trajectory)

            # some additional loss
            pre_base_dir = pre_trajectory[index] - joint_pose[:3]
            gt_base_dir = real_trajectory - joint_pose[:3]
            base_dis_loss = loss_f(pre_base_dir, gt_base_dir)

            pre_base_dir = pre_base_dir/(torch.norm(pre_base_dir,dim=-1)[:,None] + 1e-8)
            gt_base_dir = gt_base_dir/(torch.norm(gt_base_dir,dim=-1)[:,None] + 1e-8)
            base_dir_loss = -(gt_base_dir * pre_base_dir).sum()
            # trajectory_loss = direction_loss * 100 + base_dir_loss * 10 + base_dis_loss * 10
            trajectory_loss = direction_loss * 100
            if min_loss > trajectory_loss:
                min_loss = trajectory_loss
                min_step = 0
                best_joint_pose = copy.deepcopy(joint_pose)
                print(min_step,trajectory_loss.item())
            else:
                min_step += 1
                print(min_step)
            if min_step > max_iter:
                break
            optimizer.zero_grad()
            trajectory_loss.backward()
            optimizer.step()
            scheduler.step()
        return best_joint_pose.detach().numpy()

    def axis_to_trajectory(self,axis_origin,axis_direction,traj_center,degree=np.radians(60.),num=1000):
        # numpy
        # axis_direction = axis_direction
        # axis_origin = axis_origin
        # traj_center = traj_center

        delta_degree = degree/num
        start_degree = - degree/2.
        # trajectory = np.zeros((num,3))
        trajectory = torch.zeros((num,3))
        for i in range(1000):
            start_degree += i * delta_degree
            transform_rot = self.rotation_matrix(start_degree, axis_direction, axis_origin)
            # import pdb;pdb.set_trace()
            new_point = transform_rot[:3,:3] @ traj_center + transform_rot[:3,3]
            trajectory[i] = new_point
        return trajectory

    def rotation_matrix(self,angle, direction, point=None):
        import math
        sina = math.sin(angle)
        cosa = math.cos(angle)
        direction = direction[:3] / torch.norm(direction[:3])
        # rotation matrix around unit vector
        R1 = torch.diag(torch.Tensor([cosa, cosa, cosa]))
        R2 = R1 + torch.outer(direction, direction) * (1.0 - cosa)
        direction *= sina
        R = R2 + torch.Tensor(
            [
                [0.0, -direction[2], direction[1]],
                [direction[2], 0.0, -direction[0]],
                [-direction[1], direction[0], 0.0],
            ]
        )
        M = torch.eye(4)
        M[:3, :3] = R
        if point is not None:
            # rotation not around origin
            M[:3, 3] = point - R @ point
        return M

    def fine_tune_revolute_joint_pose(self,pre_joint_pose, real_trajectory, gt_joint_pose=None, max_iter=10):
        joint_pose = torch.from_numpy(pre_joint_pose)
        joint_pose[3:] = joint_pose[3:] / torch.norm(joint_pose[3:])
        # real_joint_pose = torch.from_numpy(gt_joint_pose)
        # real_joint_pose[3:] = real_joint_pose[3:] / torch.norm(real_joint_pose[3:])
        real_trajectory = torch.from_numpy(real_trajectory)
        mean_traj = real_trajectory.mean(axis=0)
        distances = torch.norm(real_trajectory - mean_traj, dim=1)
        traj_center = real_trajectory[torch.argmin(distances)]
        joint_pose.requires_grad = True
        optimizer = torch.optim.Adam([joint_pose], lr=0.0005, betas=(0.9, 0.999))
        scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=10, gamma=0.95)
        loss_f = torch.nn.L1Loss()
        min_loss = 10000
        min_step = 0
        best_joint_pose = copy.deepcopy(joint_pose)
        while True:
            pre_trajectory = self.revolute_axis_to_trajectroy(joint_pose[:3], joint_pose[3:], traj_center)
            real_trajectory_reshape = real_trajectory.reshape(real_trajectory.shape[0], 1, real_trajectory.shape[1])
            dis_matrix = torch.norm(real_trajectory_reshape - pre_trajectory, dim=2)
            indexs = torch.argmin(dis_matrix, dim=1)
            direction_loss = loss_f(pre_trajectory[indexs], real_trajectory)
            pre_base_dir = pre_trajectory[indexs] - joint_pose[:3]
            gt_base_dir = real_trajectory - joint_pose[:3]
            base_dis_loss = loss_f(pre_base_dir, gt_base_dir)
            pre_base_dir_norm = torch.norm(pre_base_dir, p=2, dim=-1)
            pre_base_dir = pre_base_dir / (pre_base_dir_norm[:, None] + 1e-8)
            gt_base_dir_norm = torch.norm(gt_base_dir, p=2, dim=-1)
            gt_base_dir = gt_base_dir / (gt_base_dir_norm[:, None] + 1e-8)
            base_dir_loss = -(pre_base_dir * gt_base_dir).sum(-1)
            base_dir_loss = base_dir_loss.sum()
            # trajectory_loss = direction_loss * 100 + base_dir_loss * 10 + base_dis_loss * 10
            trajectory_loss = direction_loss * 100
            if min_loss > trajectory_loss:
                min_loss = trajectory_loss
                min_step = 0
                best_joint_pose = copy.deepcopy(joint_pose)
                print(min_step,trajectory_loss.item())
            else:
                min_step += 1
                print(min_step)
            print(joint_pose)
            if min_step > max_iter:
                break
            optimizer.zero_grad()
            trajectory_loss.backward()
            optimizer.step()
            scheduler.step()
        best_joint_pose = best_joint_pose.detach().cpu().numpy()
        # real_joint_pose = real_joint_pose.cpu().numpy()
        # trans_error = online_dist_between_3d_lines(best_joint_pose[:3], real_joint_pose[:3], best_joint_pose[3:],
        #                                            real_joint_pose[3:])
        # direc_error = online_direction_error(best_joint_pose[3:], real_joint_pose[3:])
        return best_joint_pose, min_loss.detach().cpu().numpy()

    def revolute_axis_to_trajectroy(self,translation, direction, mean_traj, degree=0.12, num=1000):
        rotation_matrix = torch.eye(3, device=translation.device)
        z_axis = direction
        y_axis = mean_traj - translation
        x_axis = torch.cross(y_axis, z_axis)
        y_axis = torch.cross(z_axis, x_axis)
        x_axis = x_axis / torch.norm(x_axis)
        y_axis = y_axis / torch.norm(y_axis)
        z_axis = z_axis / torch.norm(z_axis)
        rotation_matrix[:3, 2] = z_axis
        rotation_matrix[:3, 1] = y_axis
        rotation_matrix[:3, 0] = x_axis
        scale_matrix = torch.norm(mean_traj - translation) * torch.eye(3, device=translation.device)
        transform_matrix = torch.eye(4, device=translation.device, dtype=translation.dtype)
        transform_matrix[:3, 3] = translation
        transform_matrix[:3, :3] = torch.mm(rotation_matrix, scale_matrix)
        transform_matrix = torch.inverse(transform_matrix)
        transformed_start = torch.mm(transform_matrix[:3, :3], mean_traj[:, None])[:, 0] + transform_matrix[:3, 3]
        trajectory = torch.zeros((num, 3), device=translation.device, dtype=translation.dtype)
        for i in range(num):
            arc_rotation_matrix = tf.rotation_matrix(angle=(60 - degree * i) / 180.0 * np.pi, direction=[0, 0, 1],
                                                     point=[0, 0, 0])
            arc_rotation_matrix = torch.from_numpy(arc_rotation_matrix[:3, :3]).to(translation.device).to(
                translation.dtype)
            trajectory[i] = torch.mm(arc_rotation_matrix, transformed_start[:, None])[:, 0]
        # transform from joint unit earth frame to original frame
        transform_matrix_inv = torch.inverse(transform_matrix)
        trajectory = torch.mm(trajectory, transform_matrix_inv[:3, :3].t()) + transform_matrix_inv[:3, 3]
        return trajectory

    def get_staight_trajectory(self,
                               joint_current,
                               ee_pose,
                               direction,
                               vel=0.01/3,
                               distance=0.2,
                               dr=0.01):
        # vel should be small enough, as there is no trajectory planning from 0 to vel
        joint_pos_current = joint_current[0]
        direction = direction / np.linalg.norm(direction)
        trajectory_N = distance / dr
        delta_t = distance / vel / trajectory_N
        waypoints = []
        home_pose = np.zeros_like(joint_pos_current)
        num = 0
        for i in range(int(trajectory_N)):
            ee_t = i * delta_t
            ee_pose[:3, 3] += direction * dr
            ee_velocity = direction * vel
            ee_twist = np.array([0,0,0])
            ee_vel = np.concatenate([ee_velocity, ee_twist])
            ee_acc = np.array([0.] *6)

            jacobian = self.robot_model.compute_jacobian(torch.from_numpy(joint_pos_current))
            jacobian_pinv = torch.pinverse(jacobian).numpy()
            jacobian = jacobian.numpy()
            qd_traj = jacobian_pinv @ ee_vel
            qdd_traj = jacobian_pinv @ ee_acc
            q_delta = qd_traj * delta_t
            joint_pos_current = joint_pos_current + q_delta

            # null space correction
            null_space_proj = np.eye(joint_pos_current.shape[0]) - jacobian_pinv @ jacobian
            q_null_err = null_space_proj @ (home_pose - joint_pos_current)
            q_null_err_norm = np.linalg.norm(q_null_err) + 1e-27  # prevent zero division
            q_null_err_clamped = (
                    q_null_err / q_null_err_norm * min(q_null_err_norm, np.linalg.norm(q_delta))
            )  # norm of correction clamped to norm of current action
            joint_pos_current = joint_pos_current + q_null_err_clamped

            in_limits = self.check_joint_limits(joint_pos_current)
            if not in_limits:
                break

            waypoint = {"time_from_start": ee_t,
                        "joint_pos": joint_pos_current,
                        "joint_vel": qd_traj,
                        "joint_acc": qdd_traj}
            waypoints.append(waypoint)
            num += 1.
        planed_dis = distance * float(num) / trajectory_N
        planed_time = ee_t
        print('planed_distance: ', np.degrees(planed_dis))
        print('planed_time: ', planed_time)
        print('planed_num: ', num)
        print('waypoints num: ', len(waypoints))
        return waypoints, planed_dis, planed_time, num


    def get_articulated_trajectory(self,
                                   joint_current,
                                   grasp_pose,
                                   axis_origin,
                                   axis_direction,
                                   vel=0.01/3,
                                   angle=-np.radians(30),
                                   dr=0.01):
        # vel should be small enough, as there is no trajectory planning from 0 to vel
        axis_direction = axis_direction/np.linalg.norm(axis_direction)
        grasp_pos = grasp_pose[:3,3]
        temp = grasp_pos - axis_origin
        grasp_proj_axis_point = np.dot(temp,axis_direction)
        grasp_radius = temp - grasp_proj_axis_point
        trajectory_length = np.abs(angle) * np.linalg.norm(grasp_radius)
        trajectory_N = int(trajectory_length/dr)
        delta_t = trajectory_length/vel/trajectory_N
        #

        delta_theta = angle / trajectory_N
        delta_pose = tf.rotation_matrix(delta_theta, axis_direction, axis_origin)
        ee_pose = grasp_pose.numpy()

        joint_pos_current = joint_current[0]
        joint_vel_current = joint_current[1]
        joint_acc_current = joint_current[2]

        waypoint = {"time_from_start":0,
                    "joint_pos":joint_pos_current,
                    "joint_vel":joint_vel_current,
                    "joint_acc":joint_acc_current}
        waypoints = []
        # waypoints.append(waypoint)

        home_pose = np.zeros_like(joint_pos_current)
        num = 0
        for i in range(trajectory_N):
            ee_pose = delta_pose @ ee_pose
            ee_t = delta_t * (i + 1)
            axis_origin2ee = ee_pose[:3,3] - axis_origin
            vel_direction = np.cross(axis_direction, axis_origin2ee) * np.sign(angle)
            vel_direction = vel_direction/np.linalg.norm(vel_direction)
            ee_velocity = vel_direction * vel
            ee_twist = axis_direction * np.sign(angle) * vel / np.linalg.norm(grasp_radius)
            ee_vel = np.concatenate([ee_velocity,ee_twist])

            ee_acc = vel**2/np.linalg.norm(grasp_radius) * (-grasp_radius)/np.linalg.norm(grasp_radius)
            ee_acc = np.concatenate([ee_acc,np.array([0.,0.,0.])])

            jacobian = self.robot_model.compute_jacobian(torch.from_numpy(joint_pos_current))
            jacobian_pinv = torch.pinverse(jacobian).numpy()
            jacobian = jacobian.numpy()
            qd_traj = jacobian_pinv @ ee_vel
            qdd_traj = jacobian_pinv @ ee_acc
            q_delta = qd_traj * delta_t
            joint_pos_current = joint_pos_current + q_delta

            # null space correction
            null_space_proj = np.eye(joint_pos_current.shape[0]) - jacobian_pinv @ jacobian
            q_null_err = null_space_proj @ (home_pose - joint_pos_current)
            q_null_err_norm = np.linalg.norm(q_null_err) + 1e-27  # prevent zero division
            q_null_err_clamped = (
                    q_null_err / q_null_err_norm * min(q_null_err_norm, np.linalg.norm(q_delta))
            )  # norm of correction clamped to norm of current action
            joint_pos_current = joint_pos_current + q_null_err_clamped

            in_limits = self.check_joint_limits(joint_pos_current)
            if not in_limits:
                break

            waypoint = {"time_from_start": ee_t,
                        "joint_pos": joint_pos_current,
                        "joint_vel": qd_traj,
                        "joint_acc": qdd_traj}
            waypoints.append(waypoint)
            num += 1.
        planed_angle = angle * float(num)/trajectory_N
        planed_time = ee_t
        print('planed_angle: ', np.degrees(planed_angle))
        print('planed_time: ', planed_time)
        print('planed_num: ', num)
        print('waypoints num: ', len(waypoints))
        return waypoints, planed_angle, planed_time, num

