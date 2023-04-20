#pragma once
#ifndef UR5TRAJOPT_HPP
#define UR5TRAJOPT_HPP

#include <iostream>


class UR5Trajopt
{
public:
  UR5Trajopt(int a);
  ~UR5Trajopt() = default;
  UR5Trajopt(const UR5Trajopt&) = default;
  UR5Trajopt& operator=(const UR5Trajopt&) = default;
  UR5Trajopt(UR5Trajopt&&) = default;
  UR5Trajopt& operator=(UR5Trajopt&&) = default;

  bool run();

private:
  
};

#endif