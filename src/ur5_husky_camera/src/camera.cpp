#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const int RATE = 50;
static const std::string OPENCV_WINDOW = "Camera 1 from Robot";

class SimpleMover {

  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub;
  ros::Subscriber image_sub;
  int RATE;

 public:

  SimpleMover(int _RATE): RATE(_RATE) {

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    image_sub = nh.subscribe("diff_drive_robot/camera1/image_raw", 10, &SimpleMover::camera_cb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~SimpleMover() {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void camera_cb(const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    show_image(cv_ptr);
  }

  void show_image(const cv_bridge::CvImagePtr cv_ptr) {
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }

  void spin() {
    ros::Rate rate(RATE);
    double time_prev = 0.0;
    double time_start = ros::Time::now().toSec();
    while (nh.ok()) {
      double t = ros::Time::now().toSec() - time_start;
      double dt = t - time_prev;
      time_prev = t;

      ROS_DEBUG("Time is: [%f]", t);

      geometry_msgs::Twist twist_msg;
      twist_msg.linear.x = 0.1;
      twist_msg.angular.z = 0.4 * sin(0.3 * t);
      cmd_vel_pub.publish(twist_msg);

      ros::spinOnce();
      rate.sleep();
    }
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_mover");

  SimpleMover simpleMover(RATE);
  simpleMover.spin();

  return 0;
}