#!/usr/bin/python3

############################################
##
##  Здесь можно управлять манипулятором (передавать 6 углов джоинтов) и размыкать гриппер на определенный угол
##
############################################



# calculation section
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from scipy.spatial.transform import Rotation
import numpy as np
import math

# default libs
import rospy
import sys
sys.path.insert(0,'/home/administrator/Repos/manipulator/geom_ws/devel/lib/python3/dist-packages')
import tf

#msg
from ur5_info.msg import MoveUR5WithGripper
from gripper_move.msg import GripperAngle

from collections import deque
import copy


class Robot():
    def __init__(self, UR_IP) -> None:
        rospy.init_node('ur5_move', anonymous=True)
        self.rate = rospy.Rate(30)
        try:
            self._rtde_c = RTDEControlInterface(
                UR_IP, RTDEControlInterface.FLAG_VERBOSE | RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
            self._rtde_r = RTDEReceiveInterface(UR_IP)
            rospy.loginfo("UR5 CONNECTED ON IP {}".format(UR_IP))
        except RuntimeError as e:
            rospy.logfatal(e)
            exit()

        rospy.Subscriber("/move_robot_delay_gripper", MoveUR5WithGripper, self.move_robot)
        rospy.on_shutdown(self.shutdown)

    def move_robot(self, msg):
        positions = msg.positions
        delay = msg.delay
        gripper_angle = msg.gripperAngle

        for i, pos in enumerate(positions):
            print(pos.position)

            self._rtde_c.moveJ(pos.position, 0.1, 10.0)
            rospy.Duration(delay[i])

            # print("Pose №", i)
            # print("Get position: ", pos)
            # print("Get delay: ", delay)

        self.rate.sleep()
        rospy.Publisher('/gripper_angle', GripperAngle, queue_size=10)


    def shutdown(self):
        rospy.sleep(1)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            

robot = Robot("192.168.131.40")
robot.spin()