#!/usr/bin/python3
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

from collections import deque
import copy

class PositionHystrory:
    def __init__(self, eefPoseGetter, maxlen=4000) -> None:
        self._eef_pose_getter = eefPoseGetter
        self.rs_frame:tuple = (rospy.get_param("~rs_frame_x", -0.03284863),
            rospy.get_param("~rs_frame_y", -0.08),
            rospy.get_param("~rs_frame_z", -0.08414625))
        self.timer = rospy.Timer(rospy.Duration(0,1000), self.callback)
        self.rate = rospy.Rate(30)

    def callback(self, arg):
        manipulatorPose = np.array(self._eef_pose_getter)
        br = tf.TransformBroadcaster()
        try:
            def ur_axis_angle_to_quat(axis_angle):
                angle = np.linalg.norm(axis_angle)
                axis_normed = [axis_angle[0]/angle, axis_angle[1]/angle, axis_angle[2]/angle]
                s = math.sin(angle/2)
                return [s*axis_normed[0], s*axis_normed[1], s*axis_normed[2], math.cos(angle/2)]
            br.sendTransform(
                (manipulatorPose[0], manipulatorPose[1], manipulatorPose[2]),
                ur_axis_angle_to_quat(manipulatorPose[3:]) ,
                rospy.Time.now(),
                "ur_gripper",
                "ur_arm_base"
            )
            br.sendTransform(
                self.rs_frame,
                list(Rotation.from_euler("xyz", [0, 0, 0]).as_quat()),
                rospy.Time.now(),
                "rs_camera",
                "ur_gripper"
            )
        except rospy.exceptions.ROSException as e:
            rospy.logwarn("ERROR PUBLISH TO TF-TREE: {}".format(e))
        rospy.loginfo_once("UR5 PUBLIHED TO TF-TREE")


class Robot():
    def __init__(self, UR_IP) -> None:
        rospy.init_node('tf_pub', anonymous=True)
        self.rate = rospy.Rate(30)
        try:
            self._rtde_c = RTDEControlInterface(
                UR_IP, RTDEControlInterface.FLAG_VERBOSE | RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
            self._rtde_r = RTDEReceiveInterface(UR_IP)
            rospy.loginfo("UR5 CONNECTED ON IP {}".format(UR_IP))
        except RuntimeError as e:
            rospy.logfatal(e)
            exit()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.sleep(1)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self._eef_hystory = PositionHystrory(self._rtde_r.getActualTCPPose())


robot = Robot("192.168.131.40")
robot.spin()