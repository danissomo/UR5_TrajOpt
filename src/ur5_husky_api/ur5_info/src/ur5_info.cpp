/////////////////////////////////////////////////////////////////
//
//  Здесь публикуется информация о джоинтах манипулятора ur5
//
/////////////////////////////////////////////////////////////////


#include <ros/ros.h>

#include <thread>
#include <chrono>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <ur_rtde/rtde_receive_interface.h>
#include <ur5_info/JointPositions.h>
#include <ur5_info/SetJointPositions.h>


using namespace ur_rtde;

#define robot_ip "192.168.131.40"


void robotJointsCallback(const ur5_info::JointPositions::ConstPtr &msg) {
  std::cout << "Robot Joint Position: ";
  for (int i = 0; i < 6; i++) {
    std::cout << msg->position[i] << " ";
  }
  std::cout << std::endl;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "ur5_info");
  ros::NodeHandle n;

  ros::Rate loop_rate(15);


  ros::Publisher joint_pub = n.advertise<ur5_info::SetJointPositions>("set_joints_positions", 10);
  ros::Subscriber joint_sub = n.subscribe<ur5_info::JointPositions>("get_joint_positions", 10, boost::bind(robotJointsCallback, _1));

  while (ros::ok) {
    ur5_info::SetJointPositions msg;

    try {
      RTDEReceiveInterface rtde_receive(robot_ip);
      ROS_INFO("Connect success!");
      std::vector<double> joint_positions = rtde_receive.getActualQ();
      msg.position = joint_positions;
      joint_pub.publish(msg);

      ROS_INFO("Get joints_positions: ");
      for (int i = 0; i < joint_positions.size(); i++) {
        std::cout << joint_positions[i] << " ";
      }

    } catch (...) {
      ROS_ERROR("Can`t connect with UR5! IP = %s", robot_ip);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
