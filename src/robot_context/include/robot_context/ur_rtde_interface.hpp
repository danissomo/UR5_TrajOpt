#pragma once
#ifndef URRTDEINTERFACE_HPP
#define URRTDEINTERFACE_HPP

#include <iostream>
#include <vector>
#include <string>

#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <ur_rtde/robotiq_gripper.h>

using namespace ur_rtde;

class URRTDEInterface
{
public:
  URRTDEInterface(std::string);
  ~URRTDEInterface() = default;

  std::shared_ptr<DashboardClient> dashboardConnect() {
    return dash_board_;
  };
  std::shared_ptr<RTDEControlInterface> rtdeControlConnect() {
    return rtde_control_;
  };
  std::shared_ptr<RTDEIOInterface> rtdeIOConnect() {
    return rtde_io_;
  };
  std::shared_ptr<RTDEReceiveInterface> rtdeReceiveConnect() {
    return rtde_receive_;
  };

private:
  std::string hostname_;
  std::shared_ptr<RTDEControlInterface> rtde_control_;
  std::shared_ptr<RTDEReceiveInterface> rtde_receive_;
  std::shared_ptr<RTDEIOInterface> rtde_io_;
  std::shared_ptr<DashboardClient> dash_board_;
};

#endif