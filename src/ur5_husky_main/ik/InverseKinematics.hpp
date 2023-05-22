#pragma once
#ifndef INVERSEKINEMATICS_HPP
#define INVERSEKINEMATICS_HPP

#include <iostream>
#include <vector>
#include <string>


class InverseKinematics
{
public:
  InverseKinematics(double, double, double);
  ~InverseKinematics() = default;

  std::vector<double> caclulate();

private:
  double x_, y_, z_;
  
};

#endif