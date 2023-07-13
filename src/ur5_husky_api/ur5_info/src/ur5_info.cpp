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
#include <ur5_info/JointVelocities.h>
#include <ur5_info/RobotState.h>


using namespace ur_rtde;

double joint_vl_plan = -1;
double joint_acc_plan = -1;


void robotJointsPosCallback(const ur5_info::JointPositions::ConstPtr &msg) {
  std::cout << "Robot Joint Position: ";
  for (int i = 0; i < 6; i++) {
    std::cout << msg->position[i] << " ";
  }
  std::cout << std::endl;
}

void robotJointsVlCallback(const ur5_info::JointVelocities::ConstPtr &msg) {
  std::cout << "Robot Joint Velocities: ";
  joint_vl_plan = msg->velocity;
  joint_acc_plan = msg->acceleration;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "ur5_info");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  std::string robot_ip = "127.0.0.1";
  nh.param("robot_ip", robot_ip, robot_ip);

  ros::Rate loop_rate(15);

  ros::Publisher joint_pub = n.advertise<ur5_info::RobotState>("/robot_state_info", 10);
  ros::Subscriber joint_sub = n.subscribe<ur5_info::JointPositions>("/get_joint_positions", 10, boost::bind(robotJointsPosCallback, _1));
  ros::Subscriber joint_vl_sub = n.subscribe<ur5_info::JointVelocities>("/joints_vel_plan", 10, boost::bind(robotJointsVlCallback, _1));

  bool connectInfo = false;
  while (ros::ok) {
    bool robotMove = false;
    ur5_info::RobotState msg;

    try {
      RTDEReceiveInterface rtde_receive(robot_ip);
      if (!connectInfo) {
        ROS_INFO("Connect success!");
        connectInfo = true;
      }

      std::vector<double> js_pos = rtde_receive.getActualQ();
      std::vector<double> js_vl = rtde_receive.getActualQd();

      for (auto vl : js_vl) {
        if (vl > 0) {
          robotMove = true;
          break;
        }
      }

      if (!robotMove) {
        joint_vl_plan = -1;
        joint_acc_plan = -1;
      }

      msg.joint_velocity_plan = joint_vl_plan;
      msg.joint_acceleration_plan = joint_acc_plan;

      msg.joint_positions = js_pos;
      msg.joint_velocities_fact = js_vl;

      joint_pub.publish(msg);

      // Print robot state info
      ROS_INFO("");
      std::cout << "Get joints_positions: ";
      for (auto pos : js_pos) {
        std::cout << pos << ", ";
      }
      std::cout << std::endl;
      std::cout << "Get joints_velosities: ";
      for (auto vl : js_vl) {
        std::cout << vl << ", ";
      }
      std::cout << std::endl;
      std::cout << "Plan valocity: " << joint_vl_plan << ", plan acceleration: " << joint_acc_plan << std::endl;

    } catch (...) {
      ROS_ERROR("Can`t connect with UR5! IP = %s", robot_ip);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
