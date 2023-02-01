#pragma once

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>

#ifndef settings
#define settings

#define robotDefaultPose "up"
#define robotDefaultLink "gripper_base_link"
#define robotDefaultJoint "gripper_finger1_joint"
#define gripperDefaultPose "open"
#define PLANNING_GROUP "manipulator"
#define GRIPPER_GROUP "gripper"
#define ROBOT_DESCRIPTION "robot_description"
#define M_PI 3.14159265358979323846

float handlePosition_x = -0.4;
float handlePosition_y = -0.2;
float handlePosition_z = 1.1;
int doorFrameHeight = 0.9144;

float gripperOpen = 0;
float gripperPickHandle = 0.5;

class SettingsClass {
    public:
        void update();
};

// Перезаписать конфиги из файла
inline void SettingsClass::update() {
    std::ifstream file("../../../data/settings.txt");
    std::string line;

    while (std::getline(file, line)) {
        std::string delimiter = "=";

        if (line.length() > 0 && line.at(0) != '#' && std::count(line.cbegin(), line.cend(), '=') == 1) {
            size_t pos = 0;
            std::string token;
            while ((pos = line.find(delimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                line.erase(0, pos + delimiter.length());
            }

            boost::trim_left(token);
            boost::trim_right(token);
            boost::trim_left(line);
            boost::trim_right(line);

            if (strcmp("handlePosition_x", token.c_str()) == 0) {
                handlePosition_x = stof(line);
            } else if (strcmp("handlePosition_y", token.c_str()) == 0) {
                handlePosition_y = stof(line);
            } else if (strcmp("handlePosition_z", token.c_str()) == 0) {
                handlePosition_z = stof(line);
            }
        }
    }
}

#endif