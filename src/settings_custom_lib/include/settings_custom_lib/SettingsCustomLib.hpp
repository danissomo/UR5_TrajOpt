#pragma once

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>

#ifndef settings_custom_lib
#define settings_custom_lib

// Настройки для MoveIt
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
float gripperPickHandle = 0.5;

class SettingsCustomLibClass {
    public:
        void update();
};

// Перезаписать конфиги из файла
inline void SettingsCustomLibClass::update() {
    std::ifstream infile("/workspace/data/settings.txt");
    std::string line;

    ROS_INFO("Settings start update...");

    while (std::getline(infile, line)) {
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
                ROS_INFO("Setting updated. New value handlePosition_x = %f", handlePosition_x);

            } else if (strcmp("handlePosition_y", token.c_str()) == 0) {
                handlePosition_y = stof(line);
                ROS_INFO("Setting updated. New value handlePosition_y = %f", handlePosition_y);

            } else if (strcmp("handlePosition_z", token.c_str()) == 0) {
                handlePosition_z = stof(line);
                ROS_INFO("Setting updated. New value handlePosition_z = %f", handlePosition_z);

            } else if (strcmp("gripperPickHandle", token.c_str()) == 0) {
                gripperPickHandle = stof(line);
                ROS_INFO("Setting updated. New value gripperPickHandle = %f", gripperPickHandle);

            }
        }
    }
}

#endif