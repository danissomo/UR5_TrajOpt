#!/usr/bin/env python3

import sys
import time
from math import sin, cos, sqrt, atan
import numpy as np
np.set_printoptions(threshold=sys.maxsize)

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import cv2
import io
import base64
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from ur5_husky_camera.msg import ImageCamera

class Camera():

    def __init__(self):
        rospy.init_node('camera', anonymous=True)

        self.rosbag = rospy.get_param("~rosbag")
        self.topic_namespace = ""
        if not self.rosbag:
            self.topic_namespace = "/pub"

        self.cv_bridge = CvBridge()

        self.ImageGripperDepth = None

        self.width = 1280
        self.height = 720

        # Images from RealSense 
        rospy.Subscriber("/pub/realsense_gripper/color/image_raw", ImageCamera, self.camera_gripper)
        rospy.Subscriber("/realsense_gripper/color/image_raw/compressed", CompressedImage, self.camera_gripper)

        rospy.Subscriber(self.topic_namespace + "/zed_node/right_raw/image_raw_color/compressed", ImageCamera, self.camera_robot)
        
        rospy.Subscriber(self.topic_namespace + "/realsense_gripper/aligned_depth_to_color/image_raw", Image, self.camera_gripper_depth)

        self.pub_gripper = rospy.Publisher('gripper_camera', ImageCamera, queue_size=10)
        self.pub_gripper_depth = rospy.Publisher('gripper_camera_depth', ImageCamera, queue_size=10)
        self.pub_robot = rospy.Publisher('robot_camera', ImageCamera, queue_size=10)
        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)


    def base64decode(self, image):
        decoded_data = base64.b64decode(image)
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
        if self.rosbag:
            try:
                cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                self.pub_gripper.publish(self.createMessage(cv_image))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            self.pub_gripper.publish(msg)
        # img = base64decode(msg.data)
        # cv2.imshow("test", img)
        # cv2.waitKey(3)

    def camera_robot(self, msg):
        if self.rosbag:
            pass
        else:
            self.pub_robot.publish(msg)

    def camera_gripper_depth(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
            msg_pub = self.createMessage(cv_image)
            self.pub_gripper_depth.publish(msg_pub)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        depth_array = np.array(cv_image, dtype=np.float32)
        ones = np.ones(depth_array.shape, dtype=np.float32)

        # cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        np.copyto(ones, depth_array, where=depth_array == 0)
        
        bl = cv2.medianBlur(ones, 5)

        hsv_min = np.array((0, 0, 0), np.uint8)
        hsv_max = np.array((0, 0, 0), np.uint8)
        
        img = cv2.cvtColor(bl, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        thresh = cv2.inRange( hsv, hsv_min, hsv_max )
        contours, hierarchy = cv2.findContours( thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # print(contours)

        for cnt in contours:
            if len(cnt)>4:
                # ellipse = cv2.fitEllipse(cnt)
                cv2.drawContours(img, cnt, -1, (0,255,0), 3)
                # cv2.ellipse(img,ellipse,(0,0,255),2)

        # print(depth_array)
        # print("=========================================================")
        
        # Process the depth image
        self.ImageGripperDepth = self.process_depth_image(img)

    def process_depth_image(self, frame):

        return frame



    def spin(self):
        start_time = time.time()
        while not rospy.is_shutdown():
            # Show Image from Camera
            if self.ImageGripperDepth is not None:
                cv2.rectangle(self.ImageGripperDepth,(5,5),(self.width-5, self.height-80),(0,255,255),5) 
                cv2.imshow("depth", self.ImageGripperDepth)
            
            cv2.waitKey(3)
            self.rate.sleep()

    def shutdown(self):
        rospy.sleep(1)


camera = Camera()
camera.spin()
