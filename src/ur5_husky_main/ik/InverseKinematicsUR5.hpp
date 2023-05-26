#pragma once
#ifndef INVERSEKINEMATICSUR5_HPP
#define INVERSEKINEMATICSUR5_HPP

#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <vector>
#include <string>

using namespace Eigen;

class InverseKinematicsUR5
{
public:
  InverseKinematicsUR5(double, double, double);
  InverseKinematicsUR5(double, double, double, double, double, double);
  ~InverseKinematicsUR5() = default;

  MatrixXd calculate();

private:
  double posX_, posY_, posZ_, roll_ = 0, pitch_ = 0, yaw_ = 0;

  
};

#endif