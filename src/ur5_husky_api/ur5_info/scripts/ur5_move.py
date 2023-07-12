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
        self.UR_IP = UR_IP


        rospy.Subscriber("/move_robot_delay_gripper", MoveUR5WithGripper, self.move_robot)
        rospy.on_shutdown(self.shutdown)

    def move_robot(self, msg):
        positions = msg.positions
        delay = msg.delay
        gripper_angle = msg.gripperAngle

        try:
            self._rtde_c = RTDEControlInterface(
                self.UR_IP, RTDEControlInterface.FLAG_VERBOSE | RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
            self._rtde_r = RTDEReceiveInterface(self.UR_IP)
            rospy.loginfo("UR5 CONNECTED ON IP {}".format(self.UR_IP))

            for i, pos in enumerate(positions):
                rospy.loginfo("Pose № {}, {}, delay: {}".format(i, pos, delay))
                self._rtde_c.moveJ(pos.position, 0.1, 10.0)
                d = rospy.Duration(delay[i])
                rospy.loginfo("Waiting {} sec...".format(delay[i]))
                rospy.sleep(d)

            self.rate.sleep()
            self._rtde_c.stopScript()
            self._rtde_c.disconnect()

        except RuntimeError as e:
            rospy.logfatal(e)
            exit()

        rospy.Publisher('/gripper_angle', GripperAngle, queue_size=10)

    def shutdown(self):
        rospy.sleep(1)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            

robot = Robot("192.168.131.40")
robot.spin()