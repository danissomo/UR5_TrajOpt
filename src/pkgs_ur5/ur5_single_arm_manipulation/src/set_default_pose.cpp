#include <ros/ros.h>
#include <ur5_single_arm_manipulation/SetDefaultPose.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>

int startMoveToPosition(ros::ServiceClient client, ur5_single_arm_manipulation::SetDefaultPose srv, std::string name) {
    srv.request.name = name;

    if (client.call(srv)) {
        ROS_INFO("Result: %s", srv.response.result.c_str());
    } else {
        ROS_ERROR("Failed to call service.");
        return 1;
    }

    return 0;
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "pose");
    
    ROS_INFO_STREAM("Writer is ready.");

    std::string name;

    ros::NodeHandle n("~");
    n.getParam("param", name);
    ROS_INFO("Got parameter: %s", name.c_str());

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<ur5_single_arm_manipulation::SetDefaultPose>("set_default_pose");
    ur5_single_arm_manipulation::SetDefaultPose srv;

    int result = 0;

    // Поменять позу
    result = startMoveToPosition(client, srv, name);

    ros::Duration(1).sleep();
    ros::spinOnce();
    
    ROS_INFO_STREAM("Publishing is finished!\n");
    return result;
}
