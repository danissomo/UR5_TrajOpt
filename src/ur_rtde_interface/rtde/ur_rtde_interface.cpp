#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include "ur_rtde_interface.hpp"

using namespace ur_rtde;

URRTDEInterface::URRTDEInterface (std::string hostname) {
  hostname_ = hostname;

  try {
    rtde_receive_ = std::make_shared<RTDEReceiveInterface>(hostname_);
  } catch (...) {
    ROS_ERROR("Can`t connect with UR5 by RTDEReceiveInterface!");
  }

  try {
    rtde_io_ = std::make_shared<RTDEIOInterface>(hostname_);
  } catch (...) {
    ROS_ERROR("Can`t connect with UR5 by RTDEIOInterface!");
  }

  try {
    rtde_control_ = std::make_shared<RTDEControlInterface>(hostname_);
  } catch (...) {
    ROS_ERROR("Can`t connect with UR5 by RTDEControlInterface!");
  }

  try {
    dash_board_ = std::make_shared<DashboardClient>(hostname_);
  } catch (...) {
    ROS_ERROR("Can`t connect with UR5 by DashboardClient!");
  }
}


