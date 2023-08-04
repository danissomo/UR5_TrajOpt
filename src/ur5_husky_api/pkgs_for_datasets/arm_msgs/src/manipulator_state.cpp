///////////////////////////////////////////////////////////////////////////////////////
//                                                                                   //
//  Здесь публикуется информация о статусе манипулятора (arm_msgs/ManipulatorState)  //
//                                                                                   //
///////////////////////////////////////////////////////////////////////////////////////


#include <ros/ros.h>

#include <thread>
#include <chrono>

#include <std_msgs/String.h>

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <arm_msgs/ManipulatorState.h>


using namespace ur_rtde;

int main(int argc, char* argv[]){
  ros::init(argc, argv, "manipulator_state_info");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  std::string robot_ip = "127.0.0.1";
  nh.param("robot_ip", robot_ip, robot_ip);

  ros::Rate loop_rate(15);

  ros::Publisher robot_pub = n.advertise<arm_msgs::ManipulatorState>("/state/arm/0/arm_state", 10);
  
  while (ros::ok) {
    bool robotMove = false;
    arm_msgs::ManipulatorState msg;

    try {
      RTDEReceiveInterface rtde_receive(robot_ip);
      RTDEControlInterface rtde_control(robot_ip);

      uint32_t rm = rtde_receive.getRobotMode();
      double robot_mode;
      memcpy(&robot_mode, &rm, sizeof(robot_mode));

      std::vector<double> joint_modes;
      std::vector<int> jm = rtde_receive.getJointMode();
      for (int i = 0; i < jm.size(); i++) {
        double jm_temp;
        memcpy(&jm_temp, &jm[i], sizeof(jm_temp));
        joint_modes.push_back(jm_temp);
      }

      uint64_t dib = rtde_receive.getActualDigitalInputBits();
      double digital_input_bits;
      memcpy(&digital_input_bits, &dib, sizeof(digital_input_bits));

      std::vector<double> empty;

      msg.q_target = rtde_receive.getTargetQ();
      msg.qd_target = rtde_receive.getTargetQd();
      msg.i_target = rtde_receive.getTargetCurrent();
      msg.m_target = rtde_receive.getTargetMoment();
      msg.tau_target = rtde_control.getJointTorques();
      msg.tool_vector_target = rtde_receive.getTargetTCPPose();
      msg.q_actual = rtde_receive.getActualQ();
      msg.qd_actual = rtde_receive.getActualQd();
      msg.i_actual = rtde_receive.getActualCurrent();
      msg.tau_actual = rtde_receive.getTargetMoment();
      msg.tcp_force = rtde_receive.getActualTCPForce();
      msg.tool_vector_actual = rtde_receive.getActualTCPPose();
      msg.tcp_speed = rtde_receive.getActualTCPSpeed();
      msg.motor_temperatures = rtde_receive.getJointTemperatures();
      msg.joint_modes = joint_modes;
      msg.controller_timer = rtde_receive.getActualExecutionTime();
      msg.qdd_target = rtde_receive.getTargetQdd();
      msg.qdd_actual = empty; // добавлю позже
      msg.tool_acc_values = rtde_receive.getActualToolAccelerometer();
      msg.robot_mode = robot_mode;
      msg.digital_input_bits = digital_input_bits;
      msg.test_value = 0.0;

      robot_pub.publish(msg);
      rtde_control.stopScript();
      rtde_control.disconnect();
      rtde_receive.disconnect();

    } catch (...) {
      ROS_ERROR("Can`t connect with UR5! IP = %s", robot_ip.c_str());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
