#!/usr/bin/env python3

import time
from math import sin, cos, sqrt, atan
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
import io
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

        self.pub_gripper = rospy.Publisher('gripper_camera', ImageCamera, queue_size=10)
        self.pub_robot = rospy.Publisher('robot_camera', ImageCamera, queue_size=10)
        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)


    def base64decode(self, image):
        decoded_data = base64.b64decode(msg.data)
        np_data = np.fromstring(decoded_data,np.uint8)
        img = cv2.imdecode(np_data,cv2.IMREAD_UNCHANGED)
        return img

    def createMessage(self, image):
        img = self.cv_bridge.cv2_to_imgmsg(image)
        _, buffer_img= cv2.imencode('.jpg', image)

        msg_img = ImageCamera()
        msg_img.data = base64.b64encode(buffer_img).decode("utf-8")
        msg_img.encoding = 'base64'
        msg_img.width = img.width
        msg_img.height = img.height
        return msg_img


    def camera_gripper(self, msg):
        self.pub_gripper.publish(msg)
        # img = base64decode(msg.data)
        # cv2.imshow("test", img)
        # cv2.waitKey(3)

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
