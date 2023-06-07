#pragma once
#ifndef TESTIK_HPP
#define TESTIK_HPP

#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <string>

using namespace Eigen;

class TestIK
{
  public:
    TestIK(std::string, double, double, double, bool);
    ~TestIK() = default;

    void ikSolverCheck(ros::Rate&, Eigen::VectorXd&);
    void getForwardKinematics(Eigen::VectorXd&);

  private:
    std::string robot_ip_;
    double ur_speed_, ur_acceleration_, ur_blend_;
    bool debug_;
};

#endif