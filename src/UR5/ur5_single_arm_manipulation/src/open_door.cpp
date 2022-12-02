#include <ros/ros.h>
#include <ur5_single_arm_manipulation/OpenDoor.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <boost/scoped_ptr.hpp>

int startOpenDoor(ros::ServiceClient client, ur5_single_arm_manipulation::OpenDoor srv) {

    if (client.call(srv)) {
        ROS_INFO("Result: %s", srv.response.result.c_str());
    } else {
        ROS_ERROR("Failed to call service.");
        return 1;
    }

    return 0;
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "open_door");
    
    ROS_INFO_STREAM("Writer 'open_door' is ready.");

    ros::NodeHandle n;

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<ur5_single_arm_manipulation::OpenDoor>("robot_open_door");
    ur5_single_arm_manipulation::OpenDoor srv;

    int result = 0;

    // Поменять угол размыкания схвата
    result = startOpenDoor(client, srv);

    ros::Duration(1).sleep();
    ros::spinOnce();
    
    ROS_INFO_STREAM("Publishing is finished!\n");
    return result;
}