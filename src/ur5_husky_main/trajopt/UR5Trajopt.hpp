#pragma once
#ifndef UR5TRAJOPT_HPP
#define UR5TRAJOPT_HPP

#include <tesseract_rosutils/plotting.h>
#include <tesseract_environment/utils.h>

#include "UR5TrajoptResponce.hpp"

#include <iostream>
#include <vector>
#include <string>

using namespace tesseract_rosutils;
using namespace tesseract_environment;

class UR5Trajopt
{
public:
  UR5Trajopt(tesseract_environment::Environment::Ptr env,
             ROSPlottingPtr plotter,
             std::vector<std::string> joint_names,
             Eigen::VectorXd joint_start_pos,
             Eigen::VectorXd joint_end_pos,
             std::vector<Eigen::VectorXd> joint_middle_pos_list);
  ~UR5Trajopt() = default;
  UR5Trajopt(const UR5Trajopt&) = default;
  UR5Trajopt& operator=(const UR5Trajopt&) = default;
  UR5Trajopt(UR5Trajopt&&) = default;
  UR5Trajopt& operator=(UR5Trajopt&&) = default;

  UR5TrajoptResponce run();

private:
  tesseract_environment::Environment::Ptr env_;
  ROSPlottingPtr plotter_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd joint_start_pos_;
  std::vector<Eigen::VectorXd> joint_middle_pos_list_;
  Eigen::VectorXd joint_end_pos_;
  
};

#endif