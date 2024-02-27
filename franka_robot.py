#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal

import numpy as np
import time
from collections import deque
from functools import partial
import threading
from scipy.spatial.transform import Rotation as Rot
from tkinter import *
import cv2
from robot import RobotViaRos
from gripper import GripperViaRos

class FrankaMoveViaRos(RobotViaRos):
    def __init__(self,record_hz=0.5):
        super(FrankaMoveViaRos,self).__init__()
        self.controller_mode={"gravity_compensation":"joint_gravity_compensation_controller",
                              "effort_trajectory":"effort_joint_trajectory_controller",
                              "position_trajectory":"position_joint_trajectory_controller"}
        
        
        self.robot_states = deque(maxlen=1)
        self.joint_states = deque(maxlen=1)
        
        self.robot_state_sub = rospy.Subscriber("/franka_state_controller/franka_states",
                                    FrankaState,
                                    # partial(self.frankastate_callback,robot_states=self.robot_states),
                                    lambda state_msg: self.robot_states.append(state_msg),
                                    queue_size=1,
                                    tcp_nodelay=True)
        
        self.joint_state_sub = rospy.Subscriber("/franka_state_controller/joint_states",
                                   JointState,
                                #    partial(self.jointstate_callback,joint_states=self.joint_states),
                                   lambda joint_msg: self.joint_states.append(joint_msg),
                                   queue_size=1,
                                   tcp_nodelay=True)
        threading.Thread(target=lambda :rospy.spin()).start()
        
        self.joint_trajectory_pub = rospy.Publisher("/effort_joint_trajectory_controller/command",
                                           JointTrajectory,queue_size=1)
        
        self.follow_joint_trajector_action = actionlib.SimpleActionClient(
            "/effort_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction)
        
        self.gripper = GripperViaRos()
        self.record_hz = record_hz
    
    def follow_jointstate_list(self,jointstate_list):
        joint_trajectory = self.convert_to_trajectory(jointstate_list)
        self.follow_joint_trajector_action.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory
        self.follow_joint_trajector_action.send_goal(goal)
        result = self.follow_joint_trajector_action.wait_for_result()
        return result
        
    
    def convert_to_trajectory(self,jointstate_list):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = jointstate_list[0].name
        start_time = jointstate_list[0].header.stamp.to_sec()
        
        # first_duration = np.abs(np.asarray(
        #     jointstate_list[0].position) - np.asarray(self.robot_states[-1].q)/0.5).max()
        first_duration = np.abs(np.asarray(
            jointstate_list[0].position) - np.asarray(self.robot_states[-1].q)).max()/ 0.5
        
        joint = JointTrajectoryPoint()
        joint.velocities = [0.]*7
        joint.accelerations = [0.]*7
        joint.time_from_start = rospy.Duration(first_duration)
        joint.positions = jointstate_list[0].position
        joint_trajectory.points.append(joint)
        
        for _joint in jointstate_list[1:]:
            duration = _joint.header.stamp.to_sec() - start_time + first_duration
            joint = JointTrajectoryPoint()
            joint.velocities = _joint.velocity
            joint.accelerations = [0.]*7
            joint.time_from_start = rospy.Duration(duration)
            joint.positions = _joint.position
            joint_trajectory.points.append(joint)
        joint_trajectory.header.stamp = rospy.Time.now()
        return joint_trajectory
    
    def record(self,):
        self.switch_to_mode('gravity_compensation')
        commands = []
        self.gripper.gripper_cmd_num = 0
        self.gripper.gripper_result_num = 0
        gripper_cmd_num = self.gripper.gripper_cmd_num
        finished = False
        cv2.namedWindow('record')
        while not finished:
            jointstate_list = []
            while True:
                time.sleep(self.record_hz)
                jointstate_list.append(self.joint_states[-1])
                # if any gripper action trigger
                if not gripper_cmd_num == self.gripper.gripper_cmd_num:
                    gripper_cmd_num = self.gripper.gripper_cmd_num
                    # save joint_trajectory, and stop record trajectory
                    commands.append(['joint_trajectory',jointstate_list])
                    # main loop: wait until gripper action finish
                    while not self.gripper.gripper_result_num == self.gripper.gripper_cmd_num:
                        pass
                    time.sleep(0.05)
                    commands.append(['gripper',self.gripper.gripper_cmd[-1]])
                    break
                
                if ord('q') == cv2.waitKey(1):
                    finished = True
                    commands.append(['joint_trajectory',jointstate_list])
                    break
        cv2.destroyAllWindows()
        return commands
    
    def execute_commands(self,commands):
        self.switch_to_mode("effort_trajectory")
        for _command in commands:
            if _command[0] == "joint_trajectory":
                jointstate_list = _command[1]
                finished = self.follow_jointstate_list(jointstate_list)
            elif _command[0] == "gripper":
                finished = self.gripper.command_gripper_cmd(_command[1])
            

if __name__ == "__main__":
    franka = FrankaMoveViaRos()
    commands = franka.record()
    # import pdb;pdb.set_trace()
    while not rospy.is_shutdown():
        franka.execute_commands(commands)
        input("continue")
    print("done")