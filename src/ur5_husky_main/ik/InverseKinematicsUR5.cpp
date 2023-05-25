#include "InverseKinematicsUR5.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#define pi 3.14159265358979323846


InverseKinematicsUR5::InverseKinematicsUR5(double x, double y, double z) {
    posX_ = x;
    posY_ = y;
    posZ_ = z;
}

std::vector<double> InverseKinematicsUR5::calculate() {

    std::cout << "X = " << posX_ << ", Y = " << posY_ << ", Z = " << posZ_ << std::endl;

    std::vector<double> a = {0, -0.425, -0.39225, 0, 0, 0};
    std::vector<double> d = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
    std::vector<double> alpha = {pi/2, 0, 0, pi/2, -pi/2, 0};
    double theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0, theta5 = 0, theta6 = 0;

    double T06[4][4];
    size_t matrix_size = 4;
    for (int i = 0; i < matrix_size; i++) {
        for (int j = 0; j < matrix_size; j++) {
            T06[i][j] = i == j;
        }
    }



    std::vector<double> joints;

    for (int i = 0; i < 6; i++) {
        joints.push_back(0);
    }



    return joints;
}