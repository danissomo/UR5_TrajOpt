#!/usr/bin/python3

############################################
##
##  Checking the connection with the robot
##
############################################

# default libs
import rospy
import sys
sys.path.insert(0,'/home/administrator/Repos/manipulator/geom_ws/devel/lib/python3/dist-packages')

#msg
from std_msgs.msg import Empty


class Robot():
    def __init__(self) -> None:
        rospy.init_node('ur5_connect', anonymous=True)
        self.rate = rospy.Rate(15)
        self.connect_pub = rospy.Publisher('/ur5_connect', Empty, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.sleep(1)

    def spin(self):

        while not rospy.is_shutdown():
            msg = Empty()
            self.connect_pub.publish(msg)
            self.rate.sleep()
            

robot = Robot()
robot.spin()