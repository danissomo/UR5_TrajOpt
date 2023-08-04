#!/usr/bin/python3

############################################
##
##  You can use a manipulator (predavat 6-year-olds) and pick a grape on the left hand
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
from ur5_info.msg import JointVelocities
from gripper_move.msg import GripperAngle

from collections import deque
import copy


class Robot():
    def __init__(self, velocity=0.1, acceleration=10.0) -> None:
        rospy.init_node('ur5_move', anonymous=True)
        self.rate = rospy.Rate(15)
        self.UR_IP = rospy.get_param('~robot_ip')
        print(self.UR_IP)
        self.velocity = velocity
        self.acceleration = acceleration

        rospy.Subscriber("/move_robot_delay_gripper", MoveUR5WithGripper, self.move_robot)
        self.gripper_pub = rospy.Publisher('/gripper_angle', GripperAngle, queue_size=10)
        self.valocity_pub = rospy.Publisher('/joints_vel_plan', JointVelocities, queue_size=10)

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
                msg_js_vel = JointVelocities()
                msg_js_vel.velocity = self.velocity
                msg_js_vel.acceleration = self.acceleration
                self.valocity_pub.publish(msg_js_vel)

                self._rtde_c.moveJ(pos.position, self.velocity, self.acceleration)
                d = rospy.Duration(delay[i])
                rospy.loginfo("Waiting {} sec...".format(delay[i]))
                rospy.sleep(d)

            self.rate.sleep()
            self._rtde_c.stopScript()
            self._rtde_c.disconnect()

        except RuntimeError as e:
            rospy.logfatal(e)
            exit()

        msg_gripper = GripperAngle()
        msg_gripper.angle = gripper_angle
        self.gripper_pub.publish(msg_gripper)

    def shutdown(self):
        rospy.sleep(1)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            

robot = Robot()
robot.spin()