#include <ros/ros.h>
#include <ur5_single_arm_manipulation/SetGripperState.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <boost/scoped_ptr.hpp>

int startMoveToPosition(ros::ServiceClient client, ur5_single_arm_manipulation::SetGripperState srv, float angular) {
    srv.request.angular = angular;

    if (client.call(srv)) {
        ROS_INFO("Result: %s", srv.response.result.c_str());
    } else {
        ROS_ERROR("Failed to call service.");
        return 1;
    }

    return 0;
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "gripper_state");
    
    ROS_INFO_STREAM("Writer is ready.");

    float angular;

    ros::NodeHandle n("~");
    n.getParam("param", angular);
    ROS_INFO("Got parameter: %s", boost::lexical_cast<std::string>(angular).c_str());

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<ur5_single_arm_manipulation::SetGripperState>("set_gripper_state");
    ur5_single_arm_manipulation::SetGripperState srv;

    int result = 0;

    // Поменять угол размыкания схвата
    result = startMoveToPosition(client, srv, angular);

    ros::Duration(1).sleep();
    ros::spinOnce();
    
    ROS_INFO_STREAM("Publishing is finished!\n");
    return result;
}