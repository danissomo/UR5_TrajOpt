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

class URRTDEInterface {
private:
    static URRTDEInterface * instance_;
    static std::mutex mutex_;

    static std::shared_ptr<RTDEControlInterface> rtde_control_;
    static std::shared_ptr<RTDEReceiveInterface> rtde_receive_;
    static std::shared_ptr<RTDEIOInterface> rtde_io_;
    static std::shared_ptr<DashboardClient> dash_board_;

    static bool robotConnect_;


protected:
    URRTDEInterface(const std::string hostname): hostname_(hostname) { }
    ~URRTDEInterface() {}
    std::string hostname_;

public:
    URRTDEInterface(URRTDEInterface &other) = delete;
    void operator=(const URRTDEInterface &) = delete;

    static URRTDEInterface *getInstance(const std::string& value);
    
    bool robotConnect() const {
        return robotConnect_;
    }
    std::string getHostname() const {
        return hostname_;
    }
    std::shared_ptr<DashboardClient> getDashboard() {
      return dash_board_;
    };
    std::shared_ptr<RTDEControlInterface> getRtdeControl() {
      return rtde_control_;
    };
    std::shared_ptr<RTDEIOInterface> getRtdeIO() {
      return rtde_io_;
    };
    std::shared_ptr<RTDEReceiveInterface> getRtdeReceive() {
      return rtde_receive_;
    };
};


#endif