
#include <ros/ros.h>

#include <ur5_single_arm_manipulation/SetPosition.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <pluginlib/class_loader.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <boost/scoped_ptr.hpp>

#include <iostream>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <cmath>

#include "MoveOperationClass.hpp"


bool setPosition(ur5_single_arm_manipulation::SetPosition::Request &req, 
                ur5_single_arm_manipulation::SetPosition::Response &res,
                MoveOperationClass *move_group,
                const robot_state::JointModelGroup *joint_model_group) {

    std::string result = "ERROR";

    geometry_msgs::Pose pose;
    bool success = true;

    pose.position.x = req.x;
    pose.position.y = req.y;
    pose.position.z = req.z;

    move_group->move->setApproximateJointValueTarget(pose,"gripper_base_link");
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    robot_state::RobotState current_state(*(move_group->move)->getCurrentState());
    move_group->move->setStartState(current_state);
    success = (move_group->move->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualizing move 1 (pose goal) %s", success ? "" : "FAILED");


    if (success) {
        ROS_INFO("Start move");

        robot_state::RobotState current_state(*(move_group->move)->getCurrentState());
        move_group->move->setStartState(current_state);
        move_group->move->move();

        std::vector<double> joints = move_group->move->getCurrentJointValues();
        result = "SUCCESS";

    } else {
        ROS_ERROR("Trajectory calculation error. A series of commands will not be sent to the robot.");
    }

    res.result = result;

    ros::spinOnce();

    return true;
}


int main(int argc, char *argv[]) {
    ROS_INFO("start:");
    ros::init(argc, argv, "move");

    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(10);
    spinner.start();

    std::string planner_plugin_name;
    if (!node_handle.getParam("planning_plugin", planner_plugin_name)) {
        ROS_FATAL_STREAM("Could not find planner plugin name");
    }

    std::string PLANNING_GROUP = "manipulator";
    MoveOperationClass *move_group = new MoveOperationClass(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_state::RobotState start_state(*(move_group->move)->getCurrentState());
    const robot_state::JointModelGroup* joint_model_group = move_group->move->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;

    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    move_group->move->setPlanningTime(60*5);
    move_group->move->setGoalTolerance(.0001);

    ros::NodeHandle n;

    // Получает позицию из position
    ros::ServiceServer setPositionService = n.advertiseService<ur5_single_arm_manipulation::SetPosition::Request, ur5_single_arm_manipulation::SetPosition::Response>
                                ("set_position", boost::bind(setPosition, _1, _2, move_group, joint_model_group));

    ros::Duration(1).sleep();
    ros::waitForShutdown();
    return 0;
}