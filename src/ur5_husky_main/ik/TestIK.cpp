#include "TestIK.hpp"
#include "InverseKinematicsUR5.hpp"

#include <ros/ros.h>

#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <eigen3/Eigen/Eigen>

#include <termios.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>


using namespace ur_rtde;


TestIK::TestIK(std::string robot_ip, double ur_speed, double ur_acceleration, double ur_blend) {
	robot_ip_ = robot_ip;
	ur_speed_ = ur_speed;
	ur_acceleration_ = ur_acceleration;
	ur_blend_ = ur_blend;
}


// Ввод символов с клавиатуры без блокировки потока ros
char getch() {
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0) {
      ROS_ERROR("tcsetattr()");
    }

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0) {
      ROS_ERROR("tcsetattr ICANON");
    }

    if(rv == -1) {
      ROS_ERROR("select");
    } else if(rv == 0) {
      ROS_INFO("no_key_pressed");
    } else {
      read(filedesc, &buff, len );
    }

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0) {
      ROS_ERROR ("tcsetattr ~ICANON");
    }
        
    return (buff);
}


void robotMove(std::vector<double> &path_pose, std::string robot_ip, double ur_speed, double ur_acceleration, double ur_blend) {
    try {

        RTDEControlInterface rtde_control(robot_ip);
        path_pose.push_back(ur_speed);
        path_pose.push_back(ur_acceleration);
        path_pose.push_back(ur_blend);

        std::vector<std::vector<double>> jointsPath;
        jointsPath.push_back(path_pose);
        rtde_control.moveJ(jointsPath);
        rtde_control.stopScript();
        rtde_control.disconnect();
    
    } catch (...) {
        ROS_ERROR("I can't connect with UR5.");
    }
}


void TestIK::ikSolverCheck(ros::Rate& loop_rate, Eigen::VectorXd& joint_start_pos) {
	  std::cout << "Проверка расчета обратной кинематики" << std::endl;

  try {
    RTDEControlInterface rtde_control(robot_ip_);
    if (rtde_control.isConnected()) {
        std::vector<double> fk = rtde_control.getForwardKinematics();
        std::cout << "Прямая кинематика: ";
        for (int i = 0; i < fk.size(); i++) {
          std::cout << fk[i] << " ";
        }
        std::cout << std::endl;

        auto begin = std::chrono::steady_clock::now();
        std::vector<double> ik = rtde_control.getInverseKinematics(fk);
        rtde_control.stopScript();
        rtde_control.disconnect();

        auto end = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
        std::cout << "Время работы алгоритма расчета IK: " << elapsed_ms.count() << " ms\n";

        std::cout << "Обратная кинематика (ur_rtde): ";
        for (int i = 0; i < ik.size(); i++) {
          std::cout << ik[i] << " ";
        }
        std::cout << std::endl;

        std::cout << "===============================" << std::endl;
        std::cout << "Обратная кинематика: " << std::endl;

        InverseKinematicsUR5 ik2(fk[0], fk[1], fk[2], fk[3], fk[4], fk[5], true);
        Eigen::MatrixXd solutions = ik2.calculateAllSolutions();

        std::cout << "\n\n-------------------------------" << std::endl;
        std::cout << "Проверка каждого решения обратной кинематики: " << std::endl;
        for (int i = 0; i < solutions.rows(); i++) {
          std::cout << "Проверяемое решение №" << i+1 << ": " << solutions.row(i) << std::endl;
          Eigen::MatrixXd fkCheck = ik2.getCheckIK(solutions.row(i));
          std::cout << "Ошибка по положению: " << fkCheck.row(0)  << std::endl;
          std::cout << "Ошибка по ориентации: " << fkCheck.row(1)  << std::endl;
          std::cout << "\n" << std::endl;
        }

        char test_robot_pose = 'n';
        std::cout << "===============================" << std::endl;
        for (int i = 0; i < solutions.rows(); i++) {
          std::cout << "Проверить решение №" << i+1 << ": XYZ = " << solutions(i, 0) << " " << solutions(i, 1) << " "<< solutions(i, 2) << ", RPY = " 
                    << solutions(i, 3) << " " << solutions(i, 4) << " "<< solutions(i, 5) << " ?" << std::endl;
          std::cout << "Введите 'y' и нажмите Enter, чтобы продолжить, 'n' - чтобы пропустить и q, чтобы завершить проверку решений..." << std::endl;
          std::cin >> test_robot_pose;

          if (test_robot_pose == 'q') {
            std::cout << "Операция прервана." << std::endl;
            break;
          } else if (test_robot_pose == 'n') {
            std::cout << "Воспроизведение решения пропущено." << std::endl;
            continue;
          }

          std::cout << "Проверка позы №" << i+1 << std::endl;

          std::vector<double> pos_vector;
          for (int j = 0; j < solutions.cols(); j++) {
            pos_vector.push_back(solutions(i, j));
          }

          robotMove(pos_vector, robot_ip_, ur_speed_, ur_acceleration_, ur_blend_);
        }

        std::cout << "===============================" << std::endl;
        std::cout << "Установите робота в стартовое положение для расчета лучшего решения. После установки введите 'y'." << std::endl;
        test_robot_pose = 'n';

        while(ros::ok()) {
          test_robot_pose = getch();

          if (test_robot_pose == 'y') {
            break;
          }

          ros::spinOnce();
          loop_rate.sleep();
        }

        Eigen::MatrixXd bestSolution = ik2.getBestSolution(solutions, joint_start_pos);
        std::cout << "===============================" << std::endl;
        std::cout << "Лучшее решение: " << std::endl;
        std::cout << bestSolution << std::endl;

        test_robot_pose = 'n';
        std::cout << "Воспроизвести лучшее решение? Нажмите y, если да, и любой другой символ, если нет. " << std::endl;
        std::cin >> test_robot_pose;

        if (test_robot_pose == 'y') {
            std::cout << "Воспроизвожу..." << std::endl;

            std::vector<double> pos_vector;
            for (int j = 0; j < bestSolution.size(); j++) {
              pos_vector.push_back(bestSolution(j));
            }

            robotMove(pos_vector, robot_ip_, ur_speed_, ur_acceleration_, ur_blend_);

        } else {
          std::cout << "Воспроизведение лучшего решения было пропущено." << std::endl;
        }

        std::cout << "Воспроизведение завершено." << std::endl;
    }

  } catch(...) {
    ROS_ERROR(" Connect error with UR5 for check inverse kinematics");
  }

}