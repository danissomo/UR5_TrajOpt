#include "InverseKinematicsUR5.hpp"

#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

using namespace Eigen;

#define pi 3.14159265358979323846


InverseKinematicsUR5::InverseKinematicsUR5(double x, double y, double z) {
    posX_ = x;
    posY_ = y;
    posZ_ = z;
}

InverseKinematicsUR5::InverseKinematicsUR5(double x, double y, double z, double roll, double pitch, double yaw) {
    posX_ = x;
    posY_ = y;
    posZ_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
}

Matrix3d euler2Quaternion(double roll, double pitch, double yaw) {
    AngleAxisd rollAngle(roll, Vector3d::UnitX());
    AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
    AngleAxisd yawAngle(yaw, Vector3d::UnitZ());

    Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}

std::vector<double> InverseKinematicsUR5::calculate() {

    std::cout << "X = " << posX_ << ", Y = " << posY_ << ", Z = " << posZ_ << std::endl;

    std::vector<double> a = {0, -0.425, -0.39225, 0, 0, 0};
    std::vector<double> d = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
    std::vector<double> alpha = {pi/2, 0, 0, pi/2, -pi/2, 0};
    double theta1[2] = {0, 0}, theta2 = 0, theta3 = 0, theta4 = 0, theta5[2] = {0, 0}, theta6 = 0;

    // Матрица вращения по углам Эйлера
    Matrix3d rotationMatrix = euler2Quaternion(roll_, pitch_, yaw_);
    std::cout << "Rotation Matrix: " << rotationMatrix << std::endl;

    Matrix4d T06;
    T06.setIdentity();
    T06.block<3,3>(0,0) = rotationMatrix;
    T06(0,3) = posX_;
    T06(1,3) = posY_;
    T06(2,3) = posZ_;


    std::cout << "Matrix: " << std::endl << T06 << std::endl;

    ////////////////////////////////// theta1 ///////////////////////////////

    Vector4d vec5(0, 0, -d[5], 1);
    Vector4d P5 = T06 * vec5;

    std::cout << "P5 === " << std::endl << P5 << std::endl;

    double theta1_ = atan2(P5(1), P5(0)) + pi/2;
    double tmp_ = acos(d[3] / sqrt(pow(P5(0), 2) + pow(P5(1), 2)));

    theta1[0] = theta1_ + tmp_;
    theta1[1] = theta1_ - tmp_;


    ////////////////////////////////// theta5 ///////////////////////////////



    std::vector<double> joints;

    for (int i = 0; i < 6; i++) {
        joints.push_back(0);
    }



    return joints;
}