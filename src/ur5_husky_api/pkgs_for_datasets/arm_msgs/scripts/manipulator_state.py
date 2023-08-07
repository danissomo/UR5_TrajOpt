#!/usr/bin/python3

##################################################################################
##                                                                              ##
##  Information about the status of the manipulator (arm_msgs/ManipulatorState) ##
##                                                                              ##
##################################################################################


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
from arm_msgs.msg import ManipulatorState

class Robot():
    def __init__(self) -> None:
        rospy.init_node('arm_msgs', anonymous=True)
        self.rate = rospy.Rate(15)
        self.UR_IP = rospy.get_param('~robot_ip')
        print(self.UR_IP)

        self.valocity_pub = rospy.Publisher('/state/arm/0/arm_state', ManipulatorState, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def state(self):
        try:
            self._rtde_c = RTDEControlInterface(
                self.UR_IP, RTDEControlInterface.FLAG_VERBOSE | RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
            self._rtde_r = RTDEReceiveInterface(self.UR_IP)
            rospy.loginfo("UR5 CONNECTED ON IP {}".format(self.UR_IP))

            msg = ManipulatorState()
            msg.q_target = self._rtde_r.getTargetQ()
            msg.qd_target = self._rtde_r.getTargetQd()
            msg.i_target = self._rtde_r.getTargetCurrent()
            msg.m_target = self._rtde_r.getTargetMoment()
            msg.tau_target = self._rtde_c.getJointTorques()
            msg.tool_vector_target = self._rtde_r.getTargetTCPPose()
            msg.q_actual = self._rtde_r.getActualQ()
            msg.qd_actual = self._rtde_r.getActualQd()
            msg.i_actual = self._rtde_r.getActualCurrent()
            msg.tau_actual = self._rtde_r.getTargetMoment()
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
            msg.test_value = 0.0;

            self.rate.sleep()
            self._rtde_c.stopScript()
            self._rtde_c.disconnect()

        except RuntimeError as e:
            rospy.logfatal(e)
            exit()

    def shutdown(self):
        rospy.sleep(1)

    def spin(self):
        while not rospy.is_shutdown():
            self.state();
            self.rate.sleep()

robot = Robot()
robot.spin()