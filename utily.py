import torch
import numpy as np
import transformations as tf
from geometry_msgs.msg import PoseStamped

def to_PoseStamped(joint_pose,stamp=None, frame_id=None, seq=None):
    axis_origin = joint_pose[:3]
    x_axis = joint_pose[3:]/np.linalg.norm(joint_pose[3:])
    y_axis = np.array([0, 1.,0])
    z_axis = np.cross(x_axis, y_axis)
    if np.linalg.norm(z_axis) < 0.02:
        y_axis = np.array([0,0,1.])
        z_axis = np.cross(x_axis, y_axis)
    pose = np.identity(4)
    pose[:3,0] = x_axis
    pose[:3,1] = y_axis
    pose[:3,2] = z_axis
    quat = tf.quaternion_from_matrix(pose)
    msg = PoseStamped()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if seq:
        msg.header.seq = seq
    msg.pose.position.x = axis_origin[0]
    msg.pose.position.y = axis_origin[1]
    msg.pose.position.z = axis_origin[2]
    msg.pose.orientation.w = quat[0]
    msg.pose.orientation.x = quat[1]
    msg.pose.orientation.y = quat[2]
    msg.pose.orientation.z = quat[3]
    return msg
