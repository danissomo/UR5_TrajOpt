#pragma once
#ifndef settings_custom_lib
#define settings_custom_lib

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>


class SettingsCustomLibClass {
    public:
        SettingsCustomLibClass();
        ~SettingsCustomLibClass() = default;
        void update();

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

        float x_pos_correct = 0.005;
        float y_pos_correct = 0.0;
        float z_pos_correct = 0.16;
        float x_orient_correct = -1.57;
        float y_orient_correct = -1.6;
        float z_orient_correct = 0;
};


#endif