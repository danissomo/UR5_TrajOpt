#pragma once
#ifndef KINEMATICSUR5_HPP
#define KINEMATICSUR5_HPP

#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <vector>
#include <string>

using namespace Eigen;

class KinematicsUR5
{
public:
  KinematicsUR5(bool);
  KinematicsUR5(double, double, double);
  KinematicsUR5(double, double, double, bool);
  KinematicsUR5(double, double, double, double, double, double);
  KinematicsUR5(double, double, double, double, double, double, bool);
  ~KinematicsUR5() = default;

  MatrixXd calculateIKAllSolutions();                           // Матрица всех решений обратной кинематики IK
  VectorXd getBestSolutionIK(VectorXd);                         // Лучшее решение обрутной кинематики IK
  VectorXd getBestSolutionIK(MatrixXd, VectorXd);               // Лучшее решение обратной кинематики IK
  MatrixXd getCheckIK(VectorXd);                                // Проверка решения обратной кинематики IK
  VectorXd getForwardkinematics(VectorXd, bool);                // Расчет прямой кинематики FK

private:
  double posX_, posY_, posZ_, roll_ = 0, pitch_ = 0, yaw_ = 0;
  bool debug_ = false;
  
};

#endif