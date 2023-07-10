#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ur5_husky_main/PoseList.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>


void chatterCallback(const ur5_husky_main::PoseList::ConstPtr& msg) {
  ROS_INFO("I heard traectory: ");
  for (int i = 0; i < msg->pose_list.size(); i++) {
    std::vector<double> pose = msg->pose_list[i].position;
    std::cout << "Pose " << i+1 << ": ";
    for (int j = 0; j < pose.size(); j++) {
      std::cout << pose[j] << " ";
    }
    std::cout << std::endl;
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_subscriber");
  ros::NodeHandle n;

  boost::function<void (const ur5_husky_main::PoseList::ConstPtr& msg)> f = boost::bind(chatterCallback, _1);
  ros::Subscriber sub = n.subscribe("trajopt_pose", 1000, f);
  ros::spin();

  return 0;
}
