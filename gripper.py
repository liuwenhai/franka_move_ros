import rospy
import actionlib

import franka_gripper.msg as grippermsg
from control_msgs.msg import GripperCommandAction,\
    GripperCommandGoal,GripperCommandActionGoal,GripperCommandActionResult
from sensor_msgs.msg import JointState

from collections import deque
import threading
from functools import partial

class GripperViaRos():
    def __init__(self,
                 server_timeout=rospy.Duration(5),
                 result_timeout=rospy.Duration(10),
                 spin_jointstate=False) -> None:
        node_name = rospy.get_name()
        if rospy.get_name() == "/unnamed":
            rospy.init_node("gripper_node")
        self.gripper_grasp_client = actionlib.SimpleActionClient("/franka_gripper/grasp",
                                                                grippermsg.GraspAction)
        self.gripper_move_client = actionlib.SimpleActionClient("/franka_gripper/move",
                                                                grippermsg.MoveAction)
        self.gripper_home_client = actionlib.SimpleActionClient("/franka_gripper/homing",
                                                                grippermsg.HomingAction)
        self.gripper_stop_client = actionlib.SimpleActionClient("/franka_gripper/stop",
                                                                grippermsg.StopAction)
        self.gripper_command_client = actionlib.SimpleActionClient("/franka_gripper/gripper_action",
                                                                GripperCommandAction)
        self.gripper_jointstate_subscriber = rospy.Subscriber("/franka_gripper/joint_states",
                                                              JointState,
                                                              self.jointstate_callback,
                                                              queue_size=1)
        
        
        # self.gripper_goal_type_dict = {
        #     '/franka_gripper/move/goal':grippermsg.MoveActionGoal,
        #     '/franka_gripper/gripper_action/goal':GripperCommandActionGoal,
        #     '/franka_gripper/homing/goal':grippermsg.HomingActionGoal,
        #     '/franka_gripper/grasp/goal':grippermsg.GraspActionGoal
        #     }
        # self.gripper_goal_to_action = {
        #     '/franka_gripper/move/goal':self.gripper_move_client,
        #     '/franka_gripper/gripper_action/goal':self.gripper_command_client,
        #     '/franka_gripper/homing/goal':self.gripper_home_client,
        #     '/franka_gripper/grasp/goal':self.gripper_grasp_client
        # }
        
        self.gripper_goal_type_dict = {
            'move':grippermsg.MoveActionGoal,
            'gripper_action':GripperCommandActionGoal,
            'homing':grippermsg.HomingActionGoal,
            'grasp':grippermsg.GraspActionGoal
            }
        
        self.gripper_result_type_dict = {
            'move':grippermsg.MoveActionResult,
            'gripper_action':GripperCommandActionResult,
            'homing':grippermsg.HomingActionResult,
            'grasp':grippermsg.GraspActionResult
            }
        
        self.gripper_goal_to_action = {
            'move':self.gripper_move_client,
            'gripper_action':self.gripper_command_client,
            'homing':self.gripper_home_client,
            'grasp':self.gripper_grasp_client
        }
        
        
        self.gripper_goal_subscriber = dict()
        [self.create_gripper_goal_subscriber(cmd_name,topic_type) 
         for cmd_name, topic_type in self.gripper_goal_type_dict.items()]
        
        self.gripper_result_subscriber = dict()
        [self.create_gripper_result_subscriber(cmd_name,topic_type) 
         for cmd_name, topic_type in self.gripper_result_type_dict.items()]
        
        
        self.waitserver_timeout = server_timeout
        self.watiresult_timeout = result_timeout
        self.jointstate = deque(maxlen=1)
        self.gripper_cmd = deque(maxlen=1)
        self.gripper_cmd_num = 0
        self.gripper_result_num = 0
        
        if spin_jointstate:
            threading.Thread(target=lambda : rospy.spin()).start()
        rospy.loginfo("Gripper client ready")
    
    def create_gripper_goal_subscriber(self,cmd_name,topic_type):
        topic_name = '/franka_gripper/%s/goal'%cmd_name
        self.gripper_goal_subscriber.update(
            {
                cmd_name:
                    rospy.Subscriber(topic_name,
                                    topic_type,
                                    partial(self.gripper_goal_callback,
                                            cmd_name=cmd_name,
                                            topic_type=topic_type),
                                    queue_size=1)
            
            })
    
    def create_gripper_result_subscriber(self,cmd_name,topic_type):
        topic_name = '/franka_gripper/%s/result'%cmd_name
        self.gripper_result_subscriber.update(
            {
                cmd_name:
                    rospy.Subscriber(topic_name,
                                    topic_type,
                                    partial(self.gripper_result_callback,
                                            cmd_name=cmd_name,
                                            topic_type=topic_type),
                                    queue_size=1)
            
            })
        
    def gripper_result_callback(self,msg,cmd_name,topic_type):
        # msg.header.stamp.to_sec()
        # msg.status
        # msg.result
        self.gripper_result_num += 1
    
    def gripper_goal_callback(self,msg,cmd_name,topic_type):
        t0 = msg.header.stamp.to_sec()
        # msg.goal_id
        # msg.goal
        self.gripper_cmd_num += 1
        while not self.gripper_cmd_num == self.gripper_result_num:
            pass
        duration = rospy.Time.now().to_sec()-t0
        self.gripper_cmd.append([self.gripper_cmd_num,cmd_name,msg.header, msg.goal, duration])
    
    def command_gripper_cmd(self,command):
        gripper_cmd_num,cmd_name,header, goal, duration = command
        action_client = self.gripper_goal_to_action[cmd_name]
        server_available=action_client.wait_for_server(self.waitserver_timeout)
        if not server_available:
            rospy.loginfo("Gripper %s server is not available"%cmd_name)
            return False
        
        action_client.send_goal(goal)
        result = self.gripper_grasp_client.wait_for_result(self.watiresult_timeout)
        return result
    
    def jointstate_callback(self,msg):
        joint_position = msg.position
        joint_velocity = msg.velocity
        joint_effort = msg.effort
        jointstate = {"position":joint_position,
                      "velocity":joint_velocity,
                      "effort":joint_effort}
        self.jointstate.append(jointstate)
        
    def grasp(self,width=0.0, epsilon_inner=0.2, epsilon_outer=0.2, speed=0.5, force=50):
        server_available=self.gripper_grasp_client.wait_for_server(self.waitserver_timeout)
        if not server_available:
            rospy.loginfo("Gripper grasp server is not available")
            return False
        grasp_goal = grippermsg.GraspGoal()
        grasp_goal.width = width
        grasp_goal.speed = speed
        grasp_goal.force = force
        grasp_goal.epsilon.inner = epsilon_inner
        grasp_goal.epsilon.outer = epsilon_outer
        self.gripper_grasp_client.send_goal(grasp_goal)
        result = self.gripper_grasp_client.wait_for_result(self.watiresult_timeout)
        return result
    
    def move(self,width = 0.08,speed = 0.5):
        server_available=self.gripper_move_client.wait_for_server(self.waitserver_timeout)
        if not server_available:
            rospy.loginfo("Gripper move server is not available")
            return False
        move_goal = grippermsg.MoveGoal()
        move_goal.width = width
        move_goal.speed = speed
        self.gripper_move_client.send_goal(move_goal)
        success = self.gripper_move_client.wait_for_result(rospy.Duration(10))
        return success
    
    def open(self):
        return self.move(width = 0.08,speed = 0.5)

    
    def home(self):
        server_available = self.gripper_home_client.wait_for_server(self.waitserver_timeout)
        if not server_available:
            rospy.loginfo("Gripper home server is not available")
            return False
        gripper_home = grippermsg.HomingGoal()
        self.gripper_home_client.send_goal(gripper_home)
        result = self.gripper_home_client.wait_for_result(self.watiresult_timeout)
        return result
    
    def stop(self):
        server_available = self.gripper_stop_client.wait_for_server(self.waitserver_timeout)
        if not server_available:
            rospy.loginfo("Gripper stop server is not available")
            return False
        gripper_stop = grippermsg.StopGoal()
        self.gripper_stop_client.send_goal(gripper_stop)
        result = self.gripper_stop_client.wait_for_result(self.watiresult_timeout)
        return result
    
    def gripper_command(self,width,max_effort):
        '''
        width : max 0.04, 0.04*2=0.08
        '''
        server_available = self.gripper_command_client.wait_for_server(self.waitserver_timeout)
        if not server_available:
            rospy.loginfo("Gripper command server is not available")
            return False
        gripper_command = GripperCommandGoal()
        gripper_command.command.position = width
        gripper_command.command.max_effort = max_effort
        self.gripper_command_client.send_goal(gripper_command)
        result_obtained = self.gripper_command_client.wait_for_result(self.watiresult_timeout)
        if result_obtained:
            result = self.gripper_command_client.get_result()
            return result
        else:
            return result_obtained


if __name__ == "__main__":
    gripper = GripperViaRos()
    # def pj():
    #     while not rospy.is_shutdown():
    #         if len(gripper.jointstate)>0 and (gripper.jointstate[0]["velocity"][0]>0 or gripper.jointstate[0]["effort"][0]>0):
    #             print(gripper.jointstate)
            
    # threading.Thread(target=pj).start()
    grasp_result = gripper.grasp()
    print(grasp_result)
    close_result = gripper.open()
    print(close_result)
    command_result=gripper.gripper_command(0.03,max_effort=10)
    print(command_result)
    # import pdb;pdb.set_trace()
    homing_result = gripper.home()
    print(homing_result)
    