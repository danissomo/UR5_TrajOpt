#!/usr/bin/env python

import time

import rospy
from sensor_msgs.msg import Image

import cv2
import base64
from cv_bridge import CvBridge, CvBridgeError
from camera_pub.msg import ImageCamera

from std_msgs.msg import String

class Camera():

    def __init__(self):
        rospy.init_node('camera', anonymous=True)

        self.cv_bridge = CvBridge()
        self.ImageGripper = None
        self.ImageRobot = None
        self.msg_img = ImageCamera()

        rospy.Subscriber("/realsense_gripper/color/image_raw", Image, self.camera_gripper)
        rospy.Subscriber("/zed_node/stereo_raw/image_raw_color", Image, self.camera_robot)


        self.pub = rospy.Publisher('/pub/realsense_gripper/color/image_raw', ImageCamera, queue_size=10)
        self.pub_robot = rospy.Publisher('/pub/zed_node/right_raw/image_raw_color/compressed', ImageCamera, queue_size=10)
        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)


    def camera_gripper(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.ImageGripper = cv_image
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def camera_robot(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.ImageRobot = cv_image
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.ImageRobot is not None:
                img = self.cv_bridge.cv2_to_imgmsg(self.ImageRobot)
                _, buffer_img= cv2.imencode('.jpg', self.ImageRobot)
                self.msg_img.data = base64.b64encode(buffer_img).decode("utf-8")
                self.msg_img.encoding = 'base64'
                self.msg_img.width = img.width
                self.msg_img.height = img.height

                self.pub_robot.publish(self.msg_img)

            if self.ImageGripper is not None:
                img = self.cv_bridge.cv2_to_imgmsg(self.ImageGripper)
                _, buffer_img= cv2.imencode('.jpg', self.ImageGripper)
                self.msg_img.data = base64.b64encode(buffer_img).decode("utf-8")
                self.msg_img.encoding = 'base64'
                self.msg_img.width = img.width
                self.msg_img.height = img.height

                self.pub.publish(self.msg_img)


    def shutdown(self):
        rospy.sleep(1)


camera = Camera()
camera.spin()

