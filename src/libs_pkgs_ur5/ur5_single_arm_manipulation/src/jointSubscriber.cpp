#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

void callback (const sensor_msgs::JointState::ConstPtr &msg) {}

int main(int argc, char **argv) {

    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;
    n.subscribe("joint_states", 1000, callback);
    ros::spin();

    return 0;
}