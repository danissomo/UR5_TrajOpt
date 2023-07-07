#!/usr/bin/python
import rospy
import actionlib
import roslib; roslib.load_manifest('ur_driver')
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import numpy as np
import time

from gripper_move.srv import *


class Gripper(object):
    def __init__(self, angle, speed = 0.10): # speed = 0.10 = 5 sm/sec
        # params
        self.pos_goal = angle
        self.pos_max = 0.085
        self.pos_min = 0.0
        self.speed = speed
        self.force = 0          # 25 Nytons (minimal force)

        if self.pos_goal < self.pos_min:
            self.pos_goal = self.pos_min
        if self.pos_goal > self.pos_max:
            self.pos_goal = self.pos_max

        # client
        self.robotiq_client = actionlib.SimpleActionClient('command_robotiq_action', CommandRobotiqGripperAction)
        self.robotiq_client.wait_for_server()
        rospy.loginfo("Connected to the gripper server")

    def move(self):
        print("Open start,  angle = %f, min_angle = %f, max_angle = %f, speed = %f", self.pos_goal, self.pos_min, self.pos_max, self.speed)
        Robotiq.goto(self.robotiq_client, pos=self.pos_goal, speed=self.speed, force=self.force, block=False)
        print("Open finish")


def handle_gripper_move(req):
    global gripper
    print(req)
    gripper = Gripper(req.angle, req.speed)
    gripper.move()
    return GripperMoveRobotResponse(True)

if __name__ == '__main__':
    rospy.init_node("gripper_controller")
    s = rospy.Service('gripper_move_robot', GripperMoveRobot, handle_gripper_move)
    rospy.spin()