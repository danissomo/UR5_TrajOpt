#!/usr/bin/python3

##################################################################################
##                                                                              ##
##  Information about the status of the manipulator (arm_msgs/ManipulatorState) ##
##                                                                              ##
##             ATTENTION! See example in the ur5_info package                   ##
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
        self.rate = rospy.Rate(30)
        self.UR_IP = rospy.get_param('~robot_ip')
        print(self.UR_IP)
        self.tau_actual = [0, 0, 0, 0, 0, 0]
        
        try:
            self._rtde_c = RTDEControlInterface(
                self.UR_IP, RTDEControlInterface.FLAG_VERBOSE | RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
            self._rtde_r = RTDEReceiveInterface(self.UR_IP)
            rospy.loginfo("UR5 CONNECTED ON IP {}".format(self.UR_IP))
        except RuntimeError as e:
            rospy.logfatal(e)
            exit()

        self.state_pub = rospy.Publisher('/state/arm/0/arm_state', ManipulatorState, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.sleep(1)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

robot = Robot()
robot.spin()