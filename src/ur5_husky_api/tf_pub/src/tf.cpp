#include <ros/ros.h>

#include <thread>
#include <chrono>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <ur_rtde/rtde_receive_interface.h>

#include <settings_custom_lib/SettingsCustomLib.hpp>


using namespace ur_rtde;
SettingsCustomLibClass settingsConfig;

void poseCallback(const std_msgs::String::Ptr& msg){
  RTDEReceiveInterface rtde_receive(robot_ip);
  std::vector<double> pose = rtde_receive.getActualTCPPose();

  std::cout << "POSE: ";
  for (int i = 0; i < pose.size(); i++) {
    std::cout << pose[i] << " ";
  }
  std::cout << std::endl;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose[0], pose[1], pose[2]));
  tf::Quaternion q;
  q.setRPY(pose[3], pose[4], pose[5]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "ur5_wrist_3_link"));
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "tf_pub_node");
  ros::NodeHandle n;

  settingsConfig.update();

  ros::Subscriber sub = n.subscribe("/pose", 10, poseCallback);
  ros::spin();
  return 0;
}
