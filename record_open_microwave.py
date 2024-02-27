#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState

import numpy as np
import time
from collections import deque
from functools import partial
import threading
from scipy.spatial.transform import Rotation as Rot
from tkinter import *
import cv2

def frankastate_callback(state_msg,robot_states):
    O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T
    robot_states.append(state_msg)

def jointstate_callback(joint_msg,joint_states):
    joint_states.append(joint_msg)

def spin():
    rospy.spin()


def record():
    global robot_state
    rs = robot_state[-1]
    O_T_EE = np.array(rs.O_T_EE).reshape(4, 4).T
    

def convert_to_trajectory(jointstate_list):
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = jointstate_list[0].name
    start_time = jointstate_list[0].header.stamp.to_sec()
    
    first_duration = np.abs(np.asarray(jointstate_list[0].position) - np.asarray(robot_states[-1].q)/0.5).max()
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

if __name__ == "__main__":
    robot_states = deque(maxlen=1)
    joint_states = deque(maxlen=1)
    rospy.init_node("record_open_microwave",anonymous=True)
    robot_state_sub = rospy.Subscriber("/franka_state_controller/franka_states",
                                   FrankaState,
                                   partial(frankastate_callback,robot_states=robot_states),
                                   queue_size=1,
                                   tcp_nodelay=True)
    
    joint_state_sub = rospy.Subscriber("/franka_state_controller/joint_states",
                                   JointState,
                                   partial(jointstate_callback,joint_states=joint_states),
                                   queue_size=1,
                                   tcp_nodelay=True)
    
    joint_trajectory_pub = rospy.Publisher("/effort_joint_trajectory_controller/command",
                                           JointTrajectory,queue_size=1)
    
    spin_t = threading.Thread(target=spin)
    spin_t.start()
    # import pdb;pdb.set_trace()
    input("start")
    cv2.namedWindow('record')
    jointstate_list = []
    while True:
        time.sleep(0.5)
        jointstate_list.append(joint_states[-1])
        if ord('q') == cv2.waitKey(1):
            break
    
    
    import pdb;pdb.set_trace()
    joint_trajectory = convert_to_trajectory(jointstate_list)
    # joint_trajectory.header.stamp = rospy.Time.now()
    joint_trajectory_pub.publish(joint_trajectory)
        
    