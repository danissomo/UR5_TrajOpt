#pragma once

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>

#ifndef settings_custom_lib
#define settings_custom_lib

float joint_middle_pos_0 = -4.662232;
float joint_middle_pos_1 = -0.382847;
float joint_middle_pos_2 = 1.830611;
float joint_middle_pos_3 = -3.041089;
float joint_middle_pos_4 = -1.641295;
float joint_middle_pos_5 = 0.020593;

float joint_middle2_pos_0 = -4.662232;
float joint_middle2_pos_1 = -1.282847;
float joint_middle2_pos_2 = 2.230611;
float joint_middle2_pos_3 = -2.541089;
float joint_middle2_pos_4 = -1.641295;
float joint_middle2_pos_5 = 0.020593;


class AddSettingsCustomLibClass {
    public:
        void update();
};

// Перезаписать конфиги из файла
inline void AddSettingsCustomLibClass::update() {
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

            if (strcmp("joint_middle_pos_0", token.c_str()) == 0) {
                joint_middle_pos_0 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle_pos_0 = %f", joint_middle_pos_0);

            } else if (strcmp("joint_middle_pos_1", token.c_str()) == 0) {
                joint_middle_pos_1 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle_pos_1 = %f", joint_middle_pos_1);

            } else if (strcmp("joint_middle_pos_2", token.c_str()) == 0) {
                joint_middle_pos_2 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle_pos_2 = %f", joint_middle_pos_2);

            } else if (strcmp("joint_middle_pos_3", token.c_str()) == 0) {
                joint_middle_pos_3 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle_pos_3 = %f", joint_middle_pos_3);

            } else if (strcmp("joint_middle_pos_4", token.c_str()) == 0) {
                joint_middle_pos_4 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle_pos_4 = %f", joint_middle_pos_4);

            } else if (strcmp("joint_middle_pos_5", token.c_str()) == 0) {
                joint_middle_pos_5 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle_pos_5 = %f", joint_middle_pos_5);

            } else if (strcmp("joint_middle2_pos_0", token.c_str()) == 0) {
                joint_middle2_pos_0 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle2_pos_0 = %f", joint_middle2_pos_0);

            } else if (strcmp("joint_middle2_pos_1", token.c_str()) == 0) {
                joint_middle2_pos_1 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle2_pos_1 = %f", joint_middle2_pos_1);

            } else if (strcmp("joint_middle2_pos_2", token.c_str()) == 0) {
                joint_middle2_pos_2 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle2_pos_2 = %f", joint_middle2_pos_2);

            } else if (strcmp("joint_middle2_pos_3", token.c_str()) == 0) {
                joint_middle2_pos_3 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle2_pos_3 = %f", joint_middle2_pos_3);

            } else if (strcmp("joint_middle2_pos_4", token.c_str()) == 0) {
                joint_middle2_pos_4 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle2_pos_4 = %f", joint_middle2_pos_4);

            } else if (strcmp("joint_middle2_pos_5", token.c_str()) == 0) {
                joint_middle2_pos_5 = stof(line);
                ROS_INFO("Setting updated. New value joint_middle2_pos_5 = %f", joint_middle2_pos_5);

            }
        }
    }
}

#endif