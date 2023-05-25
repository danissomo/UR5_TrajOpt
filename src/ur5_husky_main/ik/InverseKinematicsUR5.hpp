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
  ~InverseKinematicsUR5() = default;

  std::vector<double> calculate();

private:
  double posX_, posY_, posZ_, orientC_ = 0, orientP_ = 0, orientY_ = 0;

  
};

#endif