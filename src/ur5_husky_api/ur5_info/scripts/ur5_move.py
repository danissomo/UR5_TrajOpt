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
from arm_msgs.msg import ManipulatorState

from collections import deque
import copy
import time


class Robot():
    def __init__(self, velocity=0.1, acceleration=10.0) -> None:
        rospy.init_node('ur5_move', anonymous=True)
        self.rate = rospy.Rate(15)
        self.UR_IP = rospy.get_param('~robot_ip')
        self.velocity = velocity
        self.acceleration = acceleration
        self.tau_actual = [0, 0, 0, 0, 0, 0]
        self.num = 1

        try:
            self._rtde_c = RTDEControlInterface(
                self.UR_IP, RTDEControlInterface.FLAG_VERBOSE | RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
            self._rtde_r = RTDEReceiveInterface(self.UR_IP)
            rospy.loginfo("UR5 CONNECTED ON IP {}".format(self.UR_IP))
        except RuntimeError as e:
            rospy.logfatal(e)
            exit()

        rospy.Subscriber("/move_robot_delay_gripper", MoveUR5WithGripper, self.move_robot)
        self.gripper_pub = rospy.Publisher('/gripper_angle', GripperAngle, queue_size=10)
        self.valocity_pub = rospy.Publisher('/joints_vel_plan', JointVelocities, queue_size=10)
        self.state_pub = rospy.Publisher('/state/arm/0/arm_state', ManipulatorState, queue_size=10)

        rospy.on_shutdown(self.shutdown)

    def move_robot(self, msg):
        print(msg)
        positions = msg.positions
        delay = msg.delay
        gripper_angle = msg.gripperAngle

        for i, pos in enumerate(positions):
            rospy.loginfo("Pose {} - {}, delay: {}".format(i, pos, delay))
            msg_js_vel = JointVelocities()
            msg_js_vel.velocity = self.velocity
            msg_js_vel.acceleration = self.acceleration
            self.valocity_pub.publish(msg_js_vel)
            
            self._rtde_c.moveJ(pos.position, self.velocity, self.acceleration, True)
            d = rospy.Duration(delay[i])
            rospy.loginfo("Waiting {} sec...".format(delay[i]))
            rospy.sleep(d)

        msg_gripper = GripperAngle()
        msg_gripper.angle = gripper_angle
        self.gripper_pub.publish(msg_gripper)
    
    def manipulator_state(self):
        msg = ManipulatorState()
        msg.q_target = self._rtde_r.getTargetQ()
        msg.qd_target = self._rtde_r.getTargetQd()
        msg.i_target = self._rtde_r.getTargetCurrent()
        msg.m_target = self._rtde_r.getTargetMoment()
        msg.tau_target = self._rtde_r.getTargetMoment()
        msg.tool_vector_target = self._rtde_r.getTargetTCPPose()
        msg.q_actual = self._rtde_r.getActualQ()
        msg.qd_actual = self._rtde_r.getActualQd()
        msg.i_actual = self._rtde_r.getActualCurrent()
        msg.tau_actual = self.tau_actual
        msg.tcp_force = self._rtde_r.getActualTCPForce()
        msg.tool_vector_actual = self._rtde_r.getActualTCPPose()
        msg.tcp_speed = self._rtde_r.getActualTCPSpeed()
        msg.motor_temperatures = self._rtde_r.getJointTemperatures()
        msg.joint_modes =  [float(item) for item in self._rtde_r.getJointMode()]
        msg.controller_timer = self._rtde_r.getActualExecutionTime()
        msg.qdd_target = self._rtde_r.getTargetQdd()
        msg.qdd_actual = []
        msg.tool_acc_values = self._rtde_r.getActualToolAccelerometer()
        msg.robot_mode = self._rtde_r.getRobotMode()
        msg.digital_input_bits = float(self._rtde_r.getActualDigitalInputBits())
        msg.test_value = 0.0

        self.state_pub.publish(msg)

    def getActualJointTorques(self):
        self.tau_actual = self._rtde_c.getJointTorques()
        # rospy.sleep(rospy.Duration(10))

    def shutdown(self):
        rospy.sleep(1)

    def spin(self):
        while not rospy.is_shutdown():
            self.getActualJointTorques()
            self.manipulator_state()
            self.rate.sleep()
            

robot = Robot()
robot.spin()