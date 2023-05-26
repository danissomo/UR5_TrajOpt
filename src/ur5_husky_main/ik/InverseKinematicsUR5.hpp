#pragma once
#ifndef INVERSEKINEMATICSUR5_HPP
#define INVERSEKINEMATICSUR5_HPP

#include <iostream>
#include <vector>
#include <string>


class InverseKinematicsUR5
{
public:
  InverseKinematicsUR5(double, double, double);
  InverseKinematicsUR5(double, double, double, double, double, double);
  ~InverseKinematicsUR5() = default;

  std::vector<double> calculate();

private:
  double posX_, posY_, posZ_, roll_ = 0, pitch_ = 0, yaw_ = 0;

  
};

#endif