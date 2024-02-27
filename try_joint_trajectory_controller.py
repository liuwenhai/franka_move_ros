import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_msgs.msg import FrankaState

import numpy as np
from collections import deque
from functools import partial
import threading
from scipy.spatial.transform import Rotation as Rot

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal

def frankastate_callback(state_msg,robot_state):
    O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T
    robot_state.append(state_msg)

def spin():
    rospy.spin()

i=0
def feedback_callback(feedback):
    global i
    print(feedback.desired)
    print("desired.......................%d"%(i+1))
    print(feedback.actual)
    print("actual.......................%d"%(i+1))
    print(feedback.error)
    print("error.......................%d"%(i+1))
    i+=1

if __name__ == '__main__':
    robot_state = deque(maxlen=1)
    rospy.init_node("franka_move",anonymous=True)
    
    # /effort_joint_trajectory_controller/command
    # /position_joint_trajectory_controller/command
    joint_trajectory_pub = rospy.Publisher("/effort_joint_trajectory_controller/command",
                                           JointTrajectory,queue_size=1)
    joint_state = rospy.Subscriber("/franka_state_controller/franka_states",
                                   FrankaState,
                                   partial(frankastate_callback,robot_state=robot_state),
                                   queue_size=1,
                                   tcp_nodelay=True)
    # threading.Thread(target=spin).start()
    
    follow_joint_trajector_action = actionlib.SimpleActionClient(
            "/effort_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction)
    
    import pdb;pdb.set_trace()
    q1=[-0.1251, -0.53991, -0.00817, -2.221, 0.24551, 2.00747, 0.5867]
    q2=[-0.22535, 0.2888, -0.1851, -1.3526, 0.04490, 1.67469, 0.73048]
    q3=[-0.1372, 0.788316, -0.00479, -1.117374, 0.24547, 2.55460, 0.5866]
    rospy.sleep(1)
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = rospy.Time.now()
    joint_trajectory.joint_names = ['panda_joint1','panda_joint2',
                                    'panda_joint3','panda_joint4',
                                    'panda_joint5','panda_joint6',
                                    'panda_joint7']
    joint = JointTrajectoryPoint()
    joint.velocities = [0.]*7
    joint.accelerations = [0.]*7
    joint.time_from_start = rospy.Duration(2)
    joint.positions = q1
    joint_trajectory.points.append(joint)
    
    joint = JointTrajectoryPoint()
    joint.velocities = [0.]*7
    joint.accelerations = [0.]*7
    joint.time_from_start = rospy.Duration(5)
    joint.positions = q2
    joint_trajectory.points.append(joint)
    
    joint = JointTrajectoryPoint()
    joint.velocities = [0.]*7
    joint.accelerations = [0.]*7
    joint.time_from_start = rospy.Duration(8)
    joint.positions = q3
    joint_trajectory.points.append(joint)
    
    follow_joint_trajector_action.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = joint_trajectory
    follow_joint_trajector_action.send_goal(goal,feedback_cb=feedback_callback)
    result = follow_joint_trajector_action.wait_for_result()
    import pdb;pdb.set_trace()
    
    # joint_current = robot_state[-1].q
    # for i in range(3):
    #     joint = JointTrajectoryPoint()
    #     joint.positions = list(joint_current)
    #     joint.positions[0] += pow(-1,i)*0.25
    #     joint.velocities = [0.]*7
    #     joint.accelerations = [0.]*7
    #     joint.time_from_start = rospy.Duration(2*(i+1))
    #     joint_trajectory.points.append(joint)
    # joint_trajectory_pub.publish(joint_trajectory)
    # while not rospy.is_shutdown():
    #     joint_trajectory_pub.publish(joint_trajectory)
    #     rospy.sleep(8)
    #     joint_trajectory.header.stamp = rospy.Time.now()
    rospy.spin()
    
        
        
    
    
    
    
    