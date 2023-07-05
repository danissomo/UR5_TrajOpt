#!/usr/bin/env python3

import time
from math import sin, cos, sqrt, atan
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
import base64
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from ur5_husky_camera.srv import DefaultService, DefaultServiceResponse
from ur5_husky_camera.msg import ImageCamera

class Camera():

    def __init__(self):
        rospy.init_node('camera', anonymous=True)

        self.cv_bridge = CvBridge()

        rospy.Subscriber("/pub/realsense_gripper/color/image_raw", ImageCamera, self.camera_gripper)
        rospy.Subscriber("/pub/zed_node/right_raw/image_raw_color/compressed", ImageCamera, self.camera_robot)

        self.pub_gripper = rospy.Publisher('gripper_camera', ImageCamera, queue_size=1)
        self.rate = rospy.Rate(30)
        # self.pub_robot = rospy.Publisher('robot_camera', ImageCamera, queue_size=1)
        # self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)


    def camera_gripper(self, msg):
        self.pub_gripper.publish(msg)

    def camera_robot(self, msg):
        self.pub_robot.publish(msg)


    def spin(self):

        start_time = time.time()
        while not rospy.is_shutdown():
            self.rate.sleep()

    def shutdown(self):
        rospy.sleep(1)


camera = Camera()
camera.spin()
