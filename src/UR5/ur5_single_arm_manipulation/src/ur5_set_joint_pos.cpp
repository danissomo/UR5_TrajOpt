#include <ros/ros.h>

#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <vector>
#include <thread>
#include <chrono>
#include <termios.h>

#include <settings_custom_lib/SettingsCustomLib.hpp>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;
using namespace std::chrono;

SettingsCustomLibClass settingsConfig;


int getch() {
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


void execute() {
    try {
      RTDEReceiveInterface rtde_receive(robot_ip);
      RTDEControlInterface rtde_control(robot_ip);

      // Сначала получить данные
        std::vector<double> joint_positions = rtde_receive.getActualQ();
        ROS_INFO("Current position of UR5:");
        for (int i = 0; i < joint_positions.size(); i++) {
            ROS_INFO("%d joint: %f", i+1, joint_positions[i]);
        }

        ROS_INFO("Data got successfully");
        ROS_INFO("Execute trajectory? y/n");

        char input_simbol = 'n';
        std::cin >> input_simbol;
        if (input_simbol == 'y') {
            ROS_INFO("Loading...");
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

            ROS_INFO("Data sent successfully");

        } else {
            ROS_INFO("Data di not sent");
        }

    } catch (...) {
        ROS_ERROR("Connection or data error");
    }}


int main(int argc, char**argv) {
    settingsConfig.update();

    ros::init(argc, argv, "ur5_set_joint_pos");
    ros::NodeHandle n;

    ros::Rate loop_rate(1);
    execute();

    while(ros::ok()) {
        ROS_INFO("Hit Enter to update settings.");
        int c = getch();
        ROS_INFO("Code key = %d. Code key Enter = 10.", c);
        if (c == 10) {
            settingsConfig.update();
            execute();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}