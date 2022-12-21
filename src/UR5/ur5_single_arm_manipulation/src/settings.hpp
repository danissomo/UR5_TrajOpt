#pragma once

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


#endif
