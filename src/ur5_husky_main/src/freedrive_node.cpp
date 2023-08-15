#include <ros/ros.h>
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <settings_custom_lib/settings_custom_lib.hpp>
#include <robot_context/ur_rtde_interface.hpp>

#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;

SettingsCustomLibClass settingsConfig;

int main(int argc, char* argv[]){

  ros::init(argc, argv, "freedrive_node");
  ros::NodeHandle n;

  settingsConfig.update();

  URRTDEInterface* con =  URRTDEInterface::getInstance(settingsConfig.robot_ip);
  auto rtde_control = con->getRtdeControl();

  URRTDEInterface* con2 =  URRTDEInterface::getInstance(settingsConfig.robot_ip);
  auto rtde_control2 = con2->getRtdeControl();

  // RTDEIOInterface rtde_io(settingsConfig.robot_ip);
  // RTDEReceiveInterface rtde_receive(settingsConfig.robot_ip);
  // RTDEControlInterface rtde_control(settingsConfig.robot_ip);
  // DashboardClient dash_board(settingsConfig.robot_ip);

  bool teachModeGo(RTDEReceiveInterface&, RTDEControlInterface&, DashboardClient&);

  ros::Rate loop_rate(10);

  std::cout << "CONTROL 2=== " << rtde_control->isConnected() << std::endl;

  while(ros::ok()) {
    // bool exit = teachModeGo(rtde_receive, rtde_control, dash_board);
    // if (exit) {
    //   std::cout<<"Exit freedrive" <<std::endl;
    //   break;
    // }

    ros::spinOnce();
    loop_rate.sleep();
  }

  rtde_control->endFreedriveMode();
  rtde_control->endTeachMode();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  rtde_control->stopScript();
  rtde_control->disconnect();
  // dash_board.stop();
  // dash_board.disconnect();

  return 0;
}


bool teachModeGo(RTDEReceiveInterface &rtde_receive, RTDEControlInterface &rtde_control, DashboardClient &dash_board) {
   
  dash_board.connect();

  if (rtde_control.isConnected()) {
    std::cout << "Inter S for start FreeDrive and Q for exit" << std::endl;
    char key;
    std::cin >> key;

    std::cout << "Input comand: " << key << std::endl;

    if (key == 's' || key == 'S') {
      rtde_control.teachMode();

      std::cout << rtde_receive.getRobotMode() << "\n";
      std::cout << rtde_receive.getRobotStatus() << "\n";
      std::cout << rtde_receive.getRuntimeState() << "\n";
      std::cout << dash_board.polyscopeVersion() << "\n";
    } 

    if (key == 'q' || key == 'Q') {
      rtde_control.endTeachMode();
      std::cout << rtde_receive.getRobotStatus() << "\n";
      return true;
    }
  }

  return false;
}