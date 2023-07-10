#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_control/robotiq_gripper_client.h>

#include <ur5_husky_main/GripperService.h>

#include <thread>
#include <chrono>


typedef robotiq_2f_gripper_control::RobotiqActionClient RobotiqActionClient;

bool gripperMove(ur5_husky_main::GripperService::Request &req,
                 ur5_husky_main::GripperService::Response &res) {

  // std::string action_name = "/command_robotiq_action";  
  // bool wait_for_server = true;
  // RobotiqActionClient* gripper = new RobotiqActionClient(action_name, wait_for_server);

  // if (req.open) {
  //   gripper->open();
  // } else {
  //   gripper->close();
  // }

  return true;
}


int main(int argc, char* argv[]){

  ros::init(argc, argv, "gripper_node");
  ros::NodeHandle n;

  ros::ServiceServer gripperService = n.advertiseService<ur5_husky_main::GripperService::Request, ur5_husky_main::GripperService::Response>
                    ("gripper_move", boost::bind(gripperMove, _1, _2));

  ros::Rate loop_rate(10);

  while(ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
