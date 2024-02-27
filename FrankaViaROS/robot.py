#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState

from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import LoadController, UnloadController

import numpy as np
import time
from collections import deque
from functools import partial
import threading
from scipy.spatial.transform import Rotation as Rot
from tkinter import *
import cv2


class RobotViaRos():
    def __init__(self, ):
        self.controller_mode = dict()

        node_name = rospy.get_name()
        if rospy.get_name() == "/unnamed":
            rospy.init_node("robot_via_ros", anonymous=True)

        self.list_controllers()

        self.load_unload_controller(name='joint_gravity_compensation_controller', load=True)
        self.load_unload_controller(name='effort_joint_trajectory_controller', load=True)

        # controller manager
        '''
        /controller_manager/list_controller_types
        /controller_manager/list_controllers
        /controller_manager/load_controller
        /controller_manager/reload_controller_libraries
        /controller_manager/switch_controller
        /controller_manager/unload_controller
        '''

    def switch_to_mode(self, mode):
        '''
        mode: gravity_compensation, effort_trajectory, ''
        '''
        start_controllers = []
        stop_controllers = []
        if mode in self.controller_mode:
            name = self.controller_mode[mode]
            start_controllers.append(name)

        for _mode in self.controller_mode:
            _controller_name = self.controller_mode[_mode]
            if _controller_name in self.controller:
                _controller_status = self.controller[_controller_name]
                # "running", "initialized"
                if _controller_status == "running":
                    stop_controllers.append(_controller_name)
        # print(stop_controllers)
        result = self.switch_controller(start_controllers=start_controllers,
                                        stop_controllers=stop_controllers)
        if result:
            self.list_controllers()
        return result

    def load_unload_controller(self, name, load=True):
        if load:
            if name in self.controller:
                rospy.loginfo("%s is already loaded" % name)
                return False
            rospy.wait_for_service("controller_manager/load_controller")
            try:
                load_controller_handle = rospy.ServiceProxy(
                    "controller_manager/load_controller",
                    LoadController)
                result = load_controller_handle(name=name)
                self.list_controllers()
                return result.ok
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            if not name in self.controller:
                rospy.loginfo("%s has not loaded. Can't unload" % name)
                return False
            rospy.wait_for_service("controller_manager/unload_controller")
            try:
                unload_controller_handle = rospy.ServiceProxy(
                    "controller_manager/unload_controller",
                    UnloadController)
                result = unload_controller_handle(name=name)
                self.list_controllers()
                return result.ok
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False

    def switch_controller(self,
                          start_controllers=[],
                          stop_controllers=[],
                          strictness=2,  # BEST_EFFORT=1 or STRICT=2
                          start_asap=True,
                          timeout=2.):

        rospy.wait_for_service("/controller_manager/switch_controller")
        try:
            switch_controller_handle = rospy.ServiceProxy(
                "/controller_manager/switch_controller",
                SwitchController)
            result = switch_controller_handle(start_controllers=start_controllers,
                                              stop_controllers=stop_controllers,
                                              strictness=strictness,
                                              start_asap=start_asap,
                                              timeout=timeout)
            return result.ok

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False

    def list_controllers(self):
        '''
        return controller_list[],
        controller_list[0].name
        controller_list[0].state:"running", "initialized"
        '''
        rospy.wait_for_service("controller_manager/list_controllers")
        try:
            srv_handle = rospy.ServiceProxy(
                "controller_manager/list_controllers",
                ListControllers
            )
            controller_list = srv_handle().controller
            self.controller = dict()
            for _controller in controller_list:
                self.controller.update({_controller.name: _controller.state})
            return controller_list
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return None


if __name__ == "__main__":
    franka = RobotViaRos()
    import pdb;

    pdb.set_trace()