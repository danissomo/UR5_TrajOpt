#pragma once
#ifndef UR5TRAJOPT_HPP
#define UR5TRAJOPT_HPP

#include <tesseract_rosutils/plotting.h>
#include <tesseract_environment/utils.h>

#include <iostream>
#include <vector>
#include <string>

// #include <settings_custom_lib/SettingsCustomLib.hpp>

using namespace tesseract_rosutils;
using namespace tesseract_environment;

class UR5Trajopt
{
public:
  UR5Trajopt(tesseract_environment::Environment::Ptr env, ROSPlottingPtr plotter, std::vector<std::string> joint_names, Eigen::VectorXd joint_start_pos, Eigen::VectorXd joint_end_pos, bool ui_control);
  ~UR5Trajopt() = default;
  UR5Trajopt(const UR5Trajopt&) = default;
  UR5Trajopt& operator=(const UR5Trajopt&) = default;
  UR5Trajopt(UR5Trajopt&&) = default;
  UR5Trajopt& operator=(UR5Trajopt&&) = default;

  tesseract_common::JointTrajectory run();

private:
  tesseract_environment::Environment::Ptr env_;
  ROSPlottingPtr plotter_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd joint_start_pos_;
  Eigen::VectorXd joint_end_pos_;
  bool ui_control_;
  
};

#endif