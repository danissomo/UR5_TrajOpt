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


void set_start_pos(std::vector<double> &start_pos) {
    try {
        RTDEControlInterface rtde_control(robot_ip);

        std::vector<double> joint_q = start_pos;

        rtde_control.moveJ(joint_q);
        rtde_control.stopScript();

        ROS_INFO("UR5 reterned to start position");

    } catch (...) {
        ROS_ERROR("Connection or data error");
    }
}


void execute(bool is_start, std::vector<double> &start_pos) {
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

        if (is_start) {
            start_pos.clear();
            start_pos = joint_positions;

            ROS_INFO("Set start position: %f, %f, %f, %f, %f, %f", start_pos[0], start_pos[1], start_pos[2], start_pos[3], start_pos[4], start_pos[5]);
        }


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
            std::vector<double> joint_q = {joint_positions[0],
                                        joint_positions[1] - 0.2,
                                        joint_positions[2] + 0.2,
                                        joint_positions[3] + 0.2,
                                        joint_positions[4],
                                        joint_positions[5]};
            // std::vector<double> joint_q = {joint_test_pos_0,
            //                 joint_test_pos_1,
            //                 joint_test_pos_2,
            //                 joint_test_pos_3,
            //                 joint_test_pos_4,
            //                 joint_test_pos_5};


            rtde_control.moveJ(joint_q);
            rtde_control.stopScript();

            ROS_INFO("Data sent successfully");

        } else {
            ROS_INFO("Data did not sent");
        }

    } catch (...) {
        ROS_ERROR("Connection or data error");
    }
}


int main(int argc, char**argv) {
    settingsConfig.update();

    ros::init(argc, argv, "ur5_set_joint_pos");
    ros::NodeHandle n;

    ros::Rate loop_rate(1);

    std::vector<double> start_pos(6);

    execute(true, start_pos);

    while(ros::ok()) {
        ROS_INFO("Hit ENTER execute traectory. Or SPACE for return UR5 on start position. Or 1 for update settings from .txt file.");
        int c = getch();
        ROS_INFO("Code key = %d. Code key Enter = 10.", c);
        if (c == 10) {
            execute(false, start_pos);
        } else if (c == 32) {
            set_start_pos(start_pos);
        } else if (c == 49) {
            settingsConfig.update();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}