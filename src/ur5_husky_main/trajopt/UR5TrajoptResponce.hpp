#pragma once
#ifndef UR5TRAJOPTRESPONCE_HPP
#define UR5TRAJOPTRESPONCE_HPP

#include <tesseract_rosutils/plotting.h>
#include <tesseract_environment/utils.h>

#include <iostream>
#include <vector>
#include <string>

using namespace tesseract_rosutils;
using namespace tesseract_environment;

class UR5TrajoptResponce
{
public:
  UR5TrajoptResponce(tesseract_common::JointTrajectory  trajectory, bool isSuccessful, std::string message, double timeSecond) {
    trajectory_ = trajectory;
    isSuccessful_ = isSuccessful;
    timeSecond_ = timeSecond;
    message_ = message;
  };
  ~UR5TrajoptResponce() = default;

  tesseract_common::JointTrajectory getTrajectory() { return trajectory_; }
  double getTime() { return timeSecond_; }
  bool isSuccessful() { return isSuccessful_; }
  std::string getMessage() { return message_; }


private:
  tesseract_common::JointTrajectory trajectory_;
  double timeSecond_;
  bool isSuccessful_;
  std::string message_;
  
};

#endif