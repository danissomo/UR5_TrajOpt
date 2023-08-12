#include "ros/ros.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

// #include <ur_rtde_interface.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_node");
  ros::NodeHandle n;
  ros::spin();
  return 0;
}
