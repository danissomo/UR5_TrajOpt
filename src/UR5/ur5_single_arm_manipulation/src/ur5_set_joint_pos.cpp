#include <ros/ros.h>

#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <vector>
#include <thread>
#include <chrono>

#include <settings_custom_lib/SettingsCustomLib.hpp>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;
using namespace std::chrono;

SettingsCustomLibClass settingsConfig;


int main(int argc, char**argv) {
    settingsConfig.update();

    ros::init(argc, argv, "ur5_set_joint_pos");
    ros::NodeHandle n;

    try {
      RTDEReceiveInterface rtde_receive(robot_ip);
      RTDEControlInterface rtde_control(robot_ip);

      // Сначала получить данные
      std::vector<double> joint_positions = rtde_receive.getActualQ();
      ROS_INFO("Current position of UR5:");
      for (int i = 0; i < joint_positions.size(); i++) {
        ROS_INFO("%d joint: %f", i+1, joint_positions[i]);
      }

      // Потом отправить данные
      double velocity = 0.5;
      double acceleration = 0.5;
      double dt = 1.0/500; // 2ms
      double lookahead_time = 0.1;
      double gain = 300;
      std::vector<double> joint_q = {joint_test_pos_0,
                                    joint_test_pos_1,
                                    joint_test_pos_2,
                                    joint_test_pos_3,
                                    joint_test_pos_4,
                                    joint_test_pos_5};

      // Move to initial joint position with a regular moveJ
      rtde_control.moveJ(joint_q);

      // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
      // for (unsigned int i=0; i<1000; i++) {
      //   steady_clock::time_point t_start = rtde_control.initPeriod();
      //   rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
      //   joint_q[0] += 0.001;
      //   joint_q[1] += 0.001;
      //   rtde_control.waitPeriod(t_start);
      // }

      //rtde_control.servoStop();
      rtde_control.stopScript();

      ROS_INFO("Data received and sent successfully");

    } catch (...) {
        ROS_ERROR("Connection or data error");
    }

    ros::Duration(1).sleep();
    ros::spinOnce();

    return 0;
}