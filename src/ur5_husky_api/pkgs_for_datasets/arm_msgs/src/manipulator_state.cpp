/////////////////////////////////////////////////////////////////
//
//  Здесь публикуется информация о статусе манипулятора (arm_msgs/ManipulatorState)
//
/////////////////////////////////////////////////////////////////


#include <ros/ros.h>

#include <thread>
#include <chrono>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <ur_rtde/rtde_receive_interface.h>
#include <arm_msgs/ManipulatorState.h>


using namespace ur_rtde;

int main(int argc, char* argv[]){
  ros::init(argc, argv, "manipulator_state_info");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  std::string robot_ip = "127.0.0.1";
  nh.param("robot_ip", robot_ip, robot_ip);

  ros::Rate loop_rate(15);

  ros::Publisher joint_pub = n.advertise<arm_msgs::ManipulatorState>("/state/arm/0/arm_state", 10);
  
  while (ros::ok) {
    bool robotMove = false;
    arm_msgs::ManipulatorState msg;

    try {
      RTDEReceiveInterface rtde_receive(robot_ip);

      uint32_t rm = rtde_receive.getRobotMode();
      float robot_mode;
      memcpy(&robot_mode, &rm, sizeof(robot_mode));

      msg->q_target = rtde_receive.getTargetQ();
      msg->qd_target = rtde_receive.getTargetQd();
      msg->i_target = rtde_receive.getTargetCurrent();
      msg->m_target = rtde_receive.getTargetMoment();
      msg->tau_target = rtde_receive. // ??? Что это и откуда брать? (Надо считать)
      msg->tool_vector_target = // ???
      msg->q_actual = rtde_receive.getActualQ();
      msg->qd_actual = rtde_receive.getActualQd();
      msg->i_actual = rtde_receive.getActualCurrent();
      msg->tau_actual = rtde_receive. // аналогичный вопрос, надо считать
      msg->tcp_force = rtde_receive.getActualTCPForce();
      msg->tool_vector_actual = rtde_receive.getActualToolAccelerometer();
      msg->tcp_speed = rtde_receive.getActualTCPSpeed();
      msg->motor_temperatures = rtde_receive.getJointTemperatures();
      msg->joint_modes = rtde_receive.getJointMode();
      msg->controller_timer = rtde_receive.
      msg->qdd_target = rtde_receive.getTargetQdd();
      msg->qdd_actual = rtde_receive.
      msg->tool_acc_values = rtde_receive.
      msg->robot_mode = robot_mode;
      msg->digital_input_bits = 0.0;
      msg->test_value = 0.0;



    } catch (...) {
      ROS_ERROR("Can`t connect with UR5! IP = %s", robot_ip);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
