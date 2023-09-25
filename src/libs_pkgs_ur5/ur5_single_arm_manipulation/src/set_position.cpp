#include <ros/ros.h>
#include <ur5_single_arm_manipulation/SetPosition.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <vector>

int startMoveToPosition(ros::ServiceClient client, ur5_single_arm_manipulation::SetPosition srv, std::vector<std::string> params) {
    srv.request.x = std::stof(params[0].c_str());
    srv.request.y = std::stof(params[1].c_str());
    srv.request.z = std::stof(params[2].c_str());

    if (client.call(srv)) {
        ROS_INFO("Result: %s", srv.response.result.c_str());
    } else {
        ROS_ERROR("Failed to call service.");
        return 1;
    }

    return 0;
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "position");
    
    ROS_INFO_STREAM("Writer is ready.");

    std::vector<std::string> params;
    std::string param;

    ros::NodeHandle n("~");
    n.getParam("param", param);
    ROS_INFO("Got parameter: %s", param.c_str());

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<ur5_single_arm_manipulation::SetPosition>("set_position");
    ur5_single_arm_manipulation::SetPosition srv;

    std::stringstream data(param);

    std::string line;
    while(std::getline(data, line,' ')) {
        params.push_back(line); 
    }

    int result = 0;

    // Поменять положение
    result = startMoveToPosition(client, srv, params);

    ros::Duration(1).sleep();
    ros::spinOnce();
    
    ROS_INFO_STREAM("Publishing is finished!\n");
    return result;
}