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
  InverseKinematicsUR5(double, double, double, bool);
  InverseKinematicsUR5(double, double, double, double, double, double);
  InverseKinematicsUR5(double, double, double, double, double, double, bool);
  ~InverseKinematicsUR5() = default;

  MatrixXd calculateAllSolutions();                             // Матрица всех решений
  VectorXd getBestSolution(VectorXd);                           // Лучшее решение
  VectorXd getBestSolution(MatrixXd, VectorXd);                 // Лучшее решение
  MatrixXd getCheckIK(VectorXd);          // Проверка решения обратной кинематики
  VectorXd getForwardkinematics(VectorXd);                      // Расчет прямой кинематики       

private:
  double posX_, posY_, posZ_, roll_ = 0, pitch_ = 0, yaw_ = 0;
  bool debug_ = false;

  
};

#endif