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
    double theta1[2] = {0, 0},
           theta2 = 0,
           theta3[8] = {0, 0, 0, 0, 0, 0, 0, 0},
           theta4 = 0,
           theta5[4] = {0, 0, 0, 0},
           theta6[4] = {0, 0, 0, 0};

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
    Vector4d P05 = T06 * vec5;

    std::cout << "P05 === " << std::endl << P05 << std::endl;

    double theta1_ = atan2(P05(1), P05(0)) + pi/2;
    double acosTmp_ = acos(d[3] / sqrt(pow(P05(0), 2) + pow(P05(1), 2)));

    theta1[0] = theta1_ + acosTmp_; // положительное θ1
    theta1[1] = theta1_ - acosTmp_; // отрицательное θ1


    ////////////////////////////////// theta5 ///////////////////////////////

    Vector3d P06(posX_, posY_, posZ_);
    for (int i = 0, j = 0; i < (sizeof(theta1)/sizeof(*theta1)); i++) {
        acosTmp_ = (P06(0)*sin(theta1[i]) - P06(1)*cos(theta1[i]) - d[3])/d[5];
        theta5[j] = acosTmp_;     // для положительного θ1
        theta5[j+1] = -acosTmp_;  // для отрицательного θ1
        j += 2;
    }

    for (int k = 0; k < (sizeof(theta5)/sizeof(*theta5)); k++) {
        std::cout << "theta5 = " << theta5[k] << std::endl;
    }


    ////////////////////////////////// theta6 ///////////////////////////////

    Vector2d X06(T06(0, 0), T06(0, 1));
    Vector2d Y06(T06(1, 0), T06(1, 1));

    for (int i = 0, j = 0; i < (sizeof(theta5)/sizeof(*theta5)); i++) {
        theta6[i] = atan2((-X06(1) * sin(theta1[j]) + Y06(1))/sin(theta5[i]), (X06(0)*sin(theta1[j]) - Y06(0)*cos(theta1[j]))/sin(theta5[i]));
        if (i % 2 == 1) {
            j++; // смена знака для θ1
        }
    }


    ////////////////////////////////// theta3 ///////////////////////////////
    T14
    



    std::vector<double> joints;

    for (int i = 0; i < 6; i++) {
        joints.push_back(0);
    }



    return joints;
}