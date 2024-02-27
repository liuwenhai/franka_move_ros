from FrankaViaROS.interface import FrankaInterface
import time
import torch
import numpy as np
from tqdm import tqdm
import transformations as tf
from collections import deque
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from utily import to_PoseStamped


def pub_maker():
    marker_publisher = rospy.Publisher('/franka_grasped_object', Marker, queue_size=1)
    def viz_cube(pos):
       marker = Marker()
       marker.header.frame_id = "panda_link0"
       marker.type = marker.SPHERE
       marker.action = marker.ADD
       marker.scale.x = 0.025
       marker.scale.y = 0.025
       marker.scale.z = 0.025
       marker.color.a = 1.0
       marker.color.r = 0.0
       marker.color.g = 1.0
       marker.color.b = 0.0
       marker.pose.orientation.w = 1.0
       marker.pose.position.x = pos[0]
       marker.pose.position.y = pos[1]
       marker.pose.position.z = pos[2]
       return marker
    marker_publisher.publish(marker)

def try_microwave():
    vision_axis = deque(maxlen=1)
    # estimated_axis = deque(maxlen=1)
    rospy.init_node('articulate_adaptive_manipulation',anonymous=True)
    vision_axis_publisher = rospy.Publisher("vision/axis",PoseStamped,queue_size=1)
    # estimated_axis_publisher = rospy.Publisher("estimated/axis",PoseStamped,queue_size=1)

    def pub_axis_topic():
        while True:
            vision_axis_publisher.publish(to_PoseStamped(
                vision_axis[-1],
                stamp=rospy.Time.now(),
                frame_id='panda_link0')
            )
            # estimated_axis_publisher.publish(to_PoseStamped(
            #     estimated_axis[-1],
            #     stamp=rospy.Time.now(),
            #     frame_id='panda_link0')
            # )

    axis_publisher_thread = threading.Thread(target=pub_axis_topic,daemon=True)




    interface = FrankaInterface()
    interface.get_joint_positions()
    # cam_pos = torch.Tensor([-0.0538, -1.2344, -0.2847, -2.4828, -0.1447,  1.9838,  0.6148])
    # interface.move_to_joint_positions(cam_pos)
    # import pdb;pdb.set_trace()
    # cam_pos = torch.Tensor([-0.0538, -1.2344, -0.2847, -2.4828, -0.1447,  1.9838,  0.6148])
    # home_pose = interface.get_rest_joint()
    # current_pose = interface.get_joint_positions()

    # time_to = interface.move_to_joint_positions(home_pose)
    # # time.sleep(float(time_to))
    # time_to = interface.move_to_joint_positions(current_pose)
    # # time.sleep(float(time_to))
    # for i in range(1):
    #     interface.move_to_joint_positions(home_pose)
    #     interface.move_to_joint_positions(current_pose)
    # import pdb;pdb.set_trace()
    # time.sleep(5)
    grasp_joint = torch.Tensor([-0.284, 0.146, -0.003, -2.276, 0.1126, 3.259, 0.599])
    axis_origin = torch.Tensor([0.85, 0.15, 0.05])
    # axis_origin = torch.Tensor([0.75, 0.18, 0.06])
    # axis_direction = torch.Tensor([0.107, 0.215, 0.970])
    axis_direction = torch.Tensor([0., 0., 1.])

    vision_axis.append(np.concatenate([axis_origin.numpy(),axis_direction.numpy()]))
    # estimated_axis.append(np.concatenate([axis_origin.numpy(),axis_direction.numpy()]))
    axis_publisher_thread.start()
    # import pdb;pdb.set_trace()
    interface.move_to_joint_positions(grasp_joint)

    interface.robot_connection.switch_to_mode("gravity_compensation")
    # import pdb;pdb.set_trace()
    interface.close_gripper()
    interface.robot_connection.switch_to_mode("effort_trajectory")
    # import pdb;pdb.set_trace()
    #
    # planed_time, planed_num = interface.execute_online_articulated_trajectory(axis_origin.numpy(), axis_direction.numpy(),block_time=200)
    planed_time, planed_num = interface.execute_articulated_trajectory(axis_origin.numpy(), axis_direction.numpy(),block_time=200)
    interface.open_gripper()
    import pdb;pdb.set_trace()
    pre_joint_pose = np.concatenate([axis_origin.numpy(), axis_direction.numpy()])
    joint_pose = interface.fine_tune_revolute_joint_pose(pre_joint_pose,np.array(interface.trajectory).astype(np.float32),max_iter=10)
    estimated_axis_publisher = rospy.Publisher("estimated/axis2", PoseStamped, queue_size=1)
    estimated_axis_publisher.publish(to_PoseStamped(joint_pose[0], stamp=rospy.Time.now(), frame_id='panda_link0'))
    estimated_axis_publisher.publish(to_PoseStamped(joint1, stamp=rospy.Time.now(), frame_id='panda_link0'))
    # joint1 = np.array([0.7874264, 0.19766188, 0.12789592, 0.06077503, 0.01576262,0.93844676])
    # joint1 = np.array([0.7674264, 0.15766188, 0.10789592, 0.08077503, 0.01676262,0.90844676])
    # joint1 = np.array([0.7274264, 0.13766188, 0.0789592, 0.01077503, 0.01676262,0.95844676])
    # real_trajectory = []
    #
    # sample_points_num = planed_time / 0.5
    # delta_tim = planed_time / sample_points_num
    # for i in tqdm(range(int(sample_points_num))):
    #     t0 = time.time()
    #     ee_xyz = interface.get_ee_pose()[:3, 3].tolist()
    #     real_trajectory.append(ee_xyz)
    #     # print(estimated_axis[-1])
    #     tt = time.time() - t0
    #     if delta_tim - tt > 0:
    #         time.sleep(delta_tim - tt)
    interface.open_gripper()
    import pdb;pdb.set_trace()
    # for i in tqdm(range(int(planed_num))):
    #     t0 = time.time()
    #     ee_xyz = interface.get_ee_pose()[:3,3].tolist()
    #     real_trajectory.append(ee_xyz)
    #     best_joint_pose = interface.get_online_extimated_axis(np.array(real_trajectory),axis_origin, axis_direction)
    #     # interface.robot_connection.follow_joint_trajector_action.getResult()
    #     # interface.robot_connection.follow_joint_trajector_action.getState()
    #     # vision_axis_publisher.publish(to_PoseStamped(
    #     #     np.concatenate([axis_origin.numpy(), axis_direction.numpy()]),
    #     #     stamp=rospy.Time.now(),
    #     #     frame_id='panda_link0')
    #     # )
    #     # estimated_axis_publisher.publish(to_PoseStamped(
    #     #     best_joint_pose,
    #     #     stamp=rospy.Time.now(),
    #     #     frame_id='panda_link0')
    #     # )
    #     estimated_axis.append(best_joint_pose)
    #     print(estimated_axis[-1])
    #     tt = time.time() - t0
    #     if delta_tim - tt>0:
    #         time.sleep(delta_tim - tt)

def try_cabinet():
    vision_axis = deque(maxlen=1)
    # estimated_axis = deque(maxlen=1)
    rospy.init_node('articulate_adaptive_manipulation',anonymous=True)
    vision_axis_publisher = rospy.Publisher("vision/axis",PoseStamped,queue_size=1)
    # estimated_axis_publisher = rospy.Publisher("estimated/axis",PoseStamped,queue_size=1)

    def pub_axis_topic():
        while True:
            vision_axis_publisher.publish(to_PoseStamped(
                vision_axis[-1],
                stamp=rospy.Time.now(),
                frame_id='panda_link0')
            )
            # estimated_axis_publisher.publish(to_PoseStamped(
            #     estimated_axis[-1],
            #     stamp=rospy.Time.now(),
            #     frame_id='panda_link0')
            # )

    axis_publisher_thread = threading.Thread(target=pub_axis_topic,daemon=True)




    interface = FrankaInterface()
    # interface.get_joint_positions()
    # cam_pos = torch.Tensor([-0.0538, -1.2344, -0.2847, -2.4828, -0.1447,  1.9838,  0.6148])
    # interface.move_to_joint_positions(cam_pos)
    # import pdb;pdb.set_trace()
    # cam_pos = torch.Tensor([-0.0538, -1.2344, -0.2847, -2.4828, -0.1447,  1.9838,  0.6148])
    # home_pose = interface.get_rest_joint()
    # current_pose = interface.get_joint_positions()
    # interface.get_ee_pose()[:3,3]

    # time_to = interface.move_to_joint_positions(home_pose)
    # # time.sleep(float(time_to))
    # time_to = interface.move_to_joint_positions(current_pose)
    # # time.sleep(float(time_to))
    # for i in range(1):
    #     interface.move_to_joint_positions(home_pose)
    #     interface.move_to_joint_positions(current_pose)
    # import pdb;pdb.set_trace()
    # time.sleep(5)
    # grasp_joint = torch.Tensor([-0.284, 0.146, -0.003, -2.276, 0.1126, 3.259, 0.599])
    grasp_joint = torch.Tensor([-0.6852,  0.2649,  0.2532, -2.1116, -0.5749,  3.3175,  1.0842])
    # axis_origin = torch.Tensor([0.85, 0.15, 0.05])
    axis_origin = torch.Tensor([0.6907, 0.0686, 0.6790])
    # axis_origin = torch.Tensor([0.75, 0.18, 0.06])
    # axis_direction = torch.Tensor([0.107, 0.215, 0.970])
    axis_direction = torch.Tensor([0., 0., 1.])

    vision_axis.append(np.concatenate([axis_origin.numpy(),axis_direction.numpy()]))
    # estimated_axis.append(np.concatenate([axis_origin.numpy(),axis_direction.numpy()]))
    axis_publisher_thread.start()
    # import pdb;pdb.set_trace()
    interface.move_to_joint_positions(grasp_joint)

    interface.robot_connection.switch_to_mode("gravity_compensation")
    # import pdb;pdb.set_trace()
    interface.close_gripper()
    interface.robot_connection.switch_to_mode("effort_trajectory")
    # import pdb;pdb.set_trace()
    #
    # planed_time, planed_num = interface.execute_online_articulated_trajectory(axis_origin.numpy(), axis_direction.numpy(),block_time=200)
    planed_time, planed_num = interface.execute_articulated_trajectory(axis_origin.numpy(), axis_direction.numpy(),block_time=200)
    interface.open_gripper()
    import pdb;pdb.set_trace()


def try_drawer():
    vision_axis = deque(maxlen=1)
    # estimated_axis = deque(maxlen=1)
    rospy.init_node('articulate_adaptive_manipulation', anonymous=True)
    vision_axis_publisher = rospy.Publisher("vision/axis", PoseStamped, queue_size=1)

    # estimated_axis_publisher = rospy.Publisher("estimated/axis",PoseStamped,queue_size=1)

    def pub_axis_topic():
        while True:
            vision_axis_publisher.publish(to_PoseStamped(
                vision_axis[-1],
                stamp=rospy.Time.now(),
                frame_id='panda_link0')
            )
            # estimated_axis_publisher.publish(to_PoseStamped(
            #     estimated_axis[-1],
            #     stamp=rospy.Time.now(),
            #     frame_id='panda_link0')
            # )

    axis_publisher_thread = threading.Thread(target=pub_axis_topic, daemon=True)

    interface = FrankaInterface()
    interface.get_joint_positions()
    pre_grasp_joint = torch.Tensor([-0.2709, -0.2722,  0.0706, -2.3884, -0.0823,  3.4252, -0.7491])
    grasp_joint = torch.Tensor([-0.2706, -0.0165,  0.0889, -2.1115, -0.0823,  3.4172, -0.7491])
    axis_direction = torch.Tensor([-1.0, 0., 0.])


    # estimated_axis.append(np.concatenate([axis_origin.numpy(),axis_direction.numpy()]))

    interface.move_to_joint_positions(pre_grasp_joint)
    interface.move_to_joint_positions(grasp_joint)

    axis_origin = interface.get_ee_pose()[:3,3]
    vision_axis.append(np.concatenate([axis_origin.numpy(), axis_direction.numpy()]))
    axis_publisher_thread.start()

    interface.robot_connection.switch_to_mode("gravity_compensation")
    # import pdb;pdb.set_trace()
    interface.close_gripper()
    interface.robot_connection.switch_to_mode("effort_trajectory")
    # import pdb;pdb.set_trace()
    #
    # planed_time, planed_num = interface.execute_online_articulated_trajectory(axis_origin.numpy(), axis_direction.numpy(),block_time=200)
    planed_time, planed_num = interface.execute_straight_trajectory(axis_direction.numpy(), block_time=200)
    interface.open_gripper()
    import pdb;pdb.set_trace()


def try_online_estimate():
    rospy.init_node('articulate_adaptive_manipulation', anonymous=True)
    vision_axis_publisher = rospy.Publisher("vision/axis", PoseStamped, queue_size=1)
    estimated_axis_publisher = rospy.Publisher("estimated/axis", PoseStamped, queue_size=1)

    vision_axis = deque(maxlen=1)
    estimated_axis = deque(maxlen=1)

    grasp_joint = torch.Tensor([-0.284, 0.146, -0.003, -2.276, 0.1126, 3.259, 0.599])
    # axis_origin = torch.Tensor([0.85, 0.15, 0.05])
    axis_origin = torch.Tensor([0.8, 0.2, 0.1])
    # axis_direction = torch.Tensor([0., 0., 1.])
    axis_direction = torch.Tensor([0.0, 0.1, 0.8])
    # torch.autograd.set_detect_anomaly(True)
    def pub_axis_topic():
        # global vision_axis, estimated_axis
        while True:
            vision_axis_publisher.publish(to_PoseStamped(
                vision_axis[-1],
                stamp=rospy.Time.now(),
                frame_id='panda_link0')
            )
            estimated_axis_publisher.publish(to_PoseStamped(
                estimated_axis[-1],
                stamp=rospy.Time.now(),
                frame_id='panda_link0')
            )

    axis_publisher_thread = threading.Thread(target=pub_axis_topic, daemon=True)
    interface = FrankaInterface()
    vision_axis.append(np.concatenate([axis_origin.numpy(), axis_direction.numpy()]))
    estimated_axis.append(np.concatenate([axis_origin.numpy(), axis_direction.numpy()]))
    axis_publisher_thread.start()
    real_trajectory = np.load('trajectory.npy').astype(np.float32)
    # best_joint_pose = interface.get_online_extimated_axis(real_trajectory, axis_origin, axis_direction)
    print('start estimate')
    best_joint_pose = interface.fine_tune_revolute_joint_pose(np.concatenate([axis_origin, axis_direction]).astype(np.float32),real_trajectory)
    import pdb;pdb.set_trace()

def try_preemption_control():
    interface = FrankaInterface()
    home_pose = interface.get_rest_joint()
    current_pose = interface.get_joint_positions()
    t0 = time.time()
    time_to = interface.move_to_joint_positions(home_pose,block_time=2.)
    print('exectute time: ',time.time()-t0)
    print('time_to_go time: ',time_to)
    time_to = interface.move_to_joint_positions(current_pose, block_time=100)

# try_preemption_control()
# try_microwave()
# try_drawer()
try_cabinet()
# try_online_estimate()
