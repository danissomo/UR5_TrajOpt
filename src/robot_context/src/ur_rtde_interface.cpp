#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include "robot_context/ur_rtde_interface.hpp"

using namespace ur_rtde;


URRTDEInterface* URRTDEInterface::instance_{nullptr};
std::mutex URRTDEInterface::mutex_;

std::shared_ptr<DashboardClient> URRTDEInterface::dash_board_;
std::shared_ptr<RTDEControlInterface> URRTDEInterface::rtde_control_;
std::shared_ptr<RTDEIOInterface> URRTDEInterface::rtde_io_;
std::shared_ptr<RTDEReceiveInterface> URRTDEInterface::rtde_receive_;

URRTDEInterface *URRTDEInterface::getInstance(const std::string& hostname) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (instance_ == nullptr) {
        instance_ = new URRTDEInterface(hostname);

      try {
        rtde_receive_ = std::make_shared<RTDEReceiveInterface>(hostname);
      } catch (...) {
        ROS_ERROR("Can`t connect with UR5 by RTDEReceiveInterface!");
      }

      try {
        rtde_io_ = std::make_shared<RTDEIOInterface>(hostname);
      } catch (...) {
        ROS_ERROR("Can`t connect with UR5 by RTDEIOInterface!");
      }

      try {
        rtde_control_ = std::make_shared<RTDEControlInterface>(hostname);
      } catch (...) {
        ROS_ERROR("Can`t connect with UR5 by RTDEControlInterface!");
      }

      try {
        dash_board_ = std::make_shared<DashboardClient>(hostname);
      } catch (...) {
        ROS_ERROR("Can`t connect with UR5 by DashboardClient!");
      }
    }

    return instance_;
}

