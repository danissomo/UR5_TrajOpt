#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include "settings_custom_lib/settings_custom_lib.hpp"

SettingsCustomLibClass::SettingsCustomLibClass() { }

// Перезаписать конфиги из файла
void SettingsCustomLibClass::update() {
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

            } else if (strcmp("x_pos_correct", token.c_str()) == 0) {
                x_pos_correct = stof(line);
                ROS_INFO("Setting updated. New value x_pos_correct = %f", x_pos_correct);

            } else if (strcmp("y_pos_correct", token.c_str()) == 0) {
                y_pos_correct = stof(line);
                ROS_INFO("Setting updated. New value y_pos_correct = %f", y_pos_correct);

            } else if (strcmp("z_pos_correct", token.c_str()) == 0) {
                z_pos_correct = stof(line);
                ROS_INFO("Setting updated. New value z_pos_correct = %f", z_pos_correct);

            } else if (strcmp("x_orient_correct", token.c_str()) == 0) {
                x_orient_correct = stof(line);
                ROS_INFO("Setting updated. New value x_orient_correct = %f", x_orient_correct);

            } else if (strcmp("y_orient_correct", token.c_str()) == 0) {
                y_orient_correct = stof(line);
                ROS_INFO("Setting updated. New value y_orient_correct = %f", y_orient_correct);

            } else if (strcmp("z_orient_correct", token.c_str()) == 0) {
                z_orient_correct = stof(line);
                ROS_INFO("Setting updated. New value z_orient_correct = %f", z_orient_correct);

            } 
        }
    }
}
