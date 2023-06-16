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

float joint_start_pos_0 = 0.0;
float joint_start_pos_1 = -0.06;
float joint_start_pos_2 = -2.72;
float joint_start_pos_3 = -0.34;
float joint_start_pos_4 = 0.0;
float joint_start_pos_5 = 0.0;

float joint_end_pos_0 = 0.0;
float joint_end_pos_1 = -0.68;
float joint_end_pos_2 = -1.75;
float joint_end_pos_3 = -0.68;
float joint_end_pos_4 = 0.0;
float joint_end_pos_5 = 0.0;

std::string robot_ip = "127.0.0.1";
std::string gripper_ip = "127.0.0.1";
int gripper_port = 63352;

float joint_test_pos_0 = 0.0;
float joint_test_pos_1 = 0.0;
float joint_test_pos_2 = 0.0;
float joint_test_pos_3 = 0.0;
float joint_test_pos_4 = 0.0;
float joint_test_pos_5 = 0.0;

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

bool joint_middle_include = false;

float delay_loop_rate = 1.0;

float table_length = 0.15;
float table_width = 0.15;
float table_height = 0.15;
float table_pos_x = 0.15;
float table_pos_y = 0.0;
float table_pos_z = 0.15;

float box_length = 0.01;
float box_width = 0.005;
float box_height = 0.005;
float box_pos_x = 0.30;
float box_pos_y = 0.0;
float box_pos_z = 0.0;

std::string collision_cost_config_type = "DISCRETE_CONTINUOUS";
std::string collision_constraint_config_type = "DISCRETE_CONTINUOUS";

float ur_speed = 0.1;
float ur_acceleration = 0.1;
float ur_blend = 0.0;


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

            } else if (strcmp("joint_start_pos_0", token.c_str()) == 0) {
                joint_start_pos_0 = stof(line);
                ROS_INFO("Setting updated. New value joint_start_pos_0 = %f", joint_start_pos_0);

            } else if (strcmp("joint_start_pos_1", token.c_str()) == 0) {
                joint_start_pos_1 = stof(line);
                ROS_INFO("Setting updated. New value joint_start_pos_1 = %f", joint_start_pos_1);

            } else if (strcmp("joint_start_pos_2", token.c_str()) == 0) {
                joint_start_pos_2 = stof(line);
                ROS_INFO("Setting updated. New value joint_start_pos_2 = %f", joint_start_pos_2);

            } else if (strcmp("joint_start_pos_3", token.c_str()) == 0) {
                joint_start_pos_3 = stof(line);
                ROS_INFO("Setting updated. New value joint_start_pos_3 = %f", joint_start_pos_3);

            } else if (strcmp("joint_start_pos_4", token.c_str()) == 0) {
                joint_start_pos_4 = stof(line);
                ROS_INFO("Setting updated. New value joint_start_pos_4 = %f", joint_start_pos_4);

            } else if (strcmp("joint_start_pos_5", token.c_str()) == 0) {
                joint_start_pos_5 = stof(line);
                ROS_INFO("Setting updated. New value joint_start_pos_5 = %f", joint_start_pos_5);

            } else if (strcmp("joint_end_pos_0", token.c_str()) == 0) {
                joint_end_pos_0 = stof(line);
                ROS_INFO("Setting updated. New value joint_end_pos_0 = %f", joint_end_pos_0);

            } else if (strcmp("joint_end_pos_1", token.c_str()) == 0) {
                joint_end_pos_1 = stof(line);
                ROS_INFO("Setting updated. New value joint_end_pos_1 = %f", joint_end_pos_1);

            } else if (strcmp("joint_end_pos_2", token.c_str()) == 0) {
                joint_end_pos_2 = stof(line);
                ROS_INFO("Setting updated. New value joint_end_pos_2 = %f", joint_end_pos_2);

            } else if (strcmp("joint_end_pos_3", token.c_str()) == 0) {
                joint_end_pos_3 = stof(line);
                ROS_INFO("Setting updated. New value joint_end_pos_3 = %f", joint_end_pos_3);

            } else if (strcmp("joint_end_pos_4", token.c_str()) == 0) {
                joint_end_pos_4 = stof(line);
                ROS_INFO("Setting updated. New value joint_end_pos_4 = %f", joint_end_pos_4);

            } else if (strcmp("joint_end_pos_5", token.c_str()) == 0) {
                joint_end_pos_5 = stof(line);
                ROS_INFO("Setting updated. New value joint_end_pos_5 = %f", joint_end_pos_5);

            } else if (token == "robot_ip") {
                robot_ip = line;
                ROS_INFO("Setting updated. New value robot_ip = %s", robot_ip.c_str());

            } else if (token == "gripper_ip") {
                gripper_ip = line;
                ROS_INFO("Setting updated. New value gripper_ip = %s", gripper_ip.c_str());

            } else if (token == "gripper_port") {
                gripper_port = atoi(line.c_str());
                ROS_INFO("Setting updated. New value gripper_port = %d", gripper_port);

            } else if (strcmp("joint_test_pos_0", token.c_str()) == 0) {
                joint_test_pos_0 = stof(line);
                ROS_INFO("Setting updated. New value joint_test_pos_0 = %f", joint_test_pos_0);

            } else if (strcmp("joint_test_pos_1", token.c_str()) == 0) {
                joint_test_pos_1 = stof(line);
                ROS_INFO("Setting updated. New value joint_test_pos_1 = %f", joint_test_pos_1);

            } else if (strcmp("joint_test_pos_2", token.c_str()) == 0) {
                joint_test_pos_2 = stof(line);
                ROS_INFO("Setting updated. New value joint_test_pos_2 = %f", joint_test_pos_2);

            } else if (strcmp("joint_test_pos_3", token.c_str()) == 0) {
                joint_test_pos_3 = stof(line);
                ROS_INFO("Setting updated. New value joint_test_pos_3 = %f", joint_test_pos_3);

            } else if (strcmp("joint_test_pos_4", token.c_str()) == 0) {
                joint_test_pos_4 = stof(line);
                ROS_INFO("Setting updated. New value joint_test_pos_4 = %f", joint_test_pos_4);

            } else if (strcmp("joint_test_pos_5", token.c_str()) == 0) {
                joint_test_pos_5 = stof(line);
                ROS_INFO("Setting updated. New value joint_test_pos_5 = %f", joint_test_pos_5);

            } else if (strcmp("delay_loop_rate", token.c_str()) == 0) {
                delay_loop_rate = stof(line);
                ROS_INFO("Setting updated. New value delay_loop_rate = %f", delay_loop_rate);

            } else if (strcmp("table_length", token.c_str()) == 0) {
                table_length = stof(line);
                ROS_INFO("Setting updated. New value table_length = %f", table_length);

            } else if (strcmp("table_width", token.c_str()) == 0) {
                table_width = stof(line);
                ROS_INFO("Setting updated. New value table_width = %f", table_width);

            } else if (strcmp("table_height", token.c_str()) == 0) {
                table_height = stof(line);
                ROS_INFO("Setting updated. New value table_height = %f", table_height);

            } else if (strcmp("table_pos_x", token.c_str()) == 0) {
                table_pos_x = stof(line);
                ROS_INFO("Setting updated. New value table_pos_x = %f", table_pos_x);

            } else if (strcmp("table_pos_y", token.c_str()) == 0) {
                table_pos_y = stof(line);
                ROS_INFO("Setting updated. New value table_pos_y = %f", table_pos_y);

            } else if (strcmp("table_pos_z", token.c_str()) == 0) {
                table_pos_z = stof(line);
                ROS_INFO("Setting updated. New value table_pos_z = %f", table_pos_z);

            } else if (strcmp("box_length", token.c_str()) == 0) {
                box_length = stof(line);
                ROS_INFO("Setting updated. New value box_length = %f", box_length);

            } else if (strcmp("box_width", token.c_str()) == 0) {
                box_width = stof(line);
                ROS_INFO("Setting updated. New value box_width = %f", box_width);

            } else if (strcmp("box_height", token.c_str()) == 0) {
                box_height = stof(line);
                ROS_INFO("Setting updated. New value box_height = %f", box_height);

            } else if (strcmp("box_pos_x", token.c_str()) == 0) {
                box_pos_x = stof(line);
                ROS_INFO("Setting updated. New value box_pos_x = %f", box_pos_x);

            } else if (strcmp("box_pos_y", token.c_str()) == 0) {
                box_pos_y = stof(line);
                ROS_INFO("Setting updated. New value box_pos_y = %f", box_pos_y);

            } else if (strcmp("box_pos_z", token.c_str()) == 0) {
                box_pos_z = stof(line);
                ROS_INFO("Setting updated. New value box_pos_z = %f", box_pos_z);

            } else if (token == "collision_cost_config_type") {
                collision_cost_config_type = line;
                ROS_INFO("Setting updated. New value collision_cost_config_type = %s", robot_ip.c_str());

            } else if (token == "collision_constraint_config_type") {
                collision_constraint_config_type = line;
                ROS_INFO("Setting updated. New value collision_constraint_config_type = %s", robot_ip.c_str());
            } else if (strcmp("ur_speed", token.c_str()) == 0) {
                ur_speed = stof(line);
                ROS_INFO("Setting updated. New value ur_speed = %f", ur_speed);

            } else if (strcmp("ur_acceleration", token.c_str()) == 0) {
                ur_acceleration = stof(line);
                ROS_INFO("Setting updated. New value ur_acceleration = %f", ur_acceleration);

            } else if (strcmp("ur_blend", token.c_str()) == 0) {
                ur_blend = stof(line);
                ROS_INFO("Setting updated. New value ur_blend = %f", ur_blend);

            } else if (strcmp("joint_middle_pos_0", token.c_str()) == 0) {
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

            } else if (strcmp("joint_middle_include", token.c_str()) == 0) {
                joint_middle_include = (strcmp("true", line.c_str()) == 0);
                ROS_INFO("Setting updated. New value joint_middle_include = %f", joint_middle_include);

            }
        }
    }
}

#endif