#include <tesseract_examples/ur5_trajopt.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>

#include <settings_custom_lib/SettingsCustomLib.hpp>

using namespace ur_rtde;

using namespace tesseract_examples;
using namespace tesseract_rosutils;

SettingsCustomLibClass settingsConfig;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5_trajopt_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  bool debug = false;
  bool sim_robot = true;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("debug", debug, debug);
  pnh.param("sim_robot", sim_robot, sim_robot);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  settingsConfig.update();

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator)) {
    exit(1);
  }

  // Create monitor
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz) {
    monitor->startPublishingEnvironment();
  }

  ROSPlottingPtr plotter;
  if (plotting) {
    plotter = std::make_shared<ROSPlotting>(env->getSceneGraph()->getRoot());
  }

  Eigen::VectorXd joint_start_pos(6);
  joint_start_pos(0) = joint_start_pos_0;
  joint_start_pos(1) = joint_start_pos_1;
  joint_start_pos(2) = joint_start_pos_2;
  joint_start_pos(3) = joint_start_pos_3;
  joint_start_pos(4) = joint_start_pos_4;
  joint_start_pos(5) = joint_start_pos_5;

  Eigen::VectorXd joint_end_pos(6);
  joint_end_pos(0) = joint_end_pos_0;
  joint_end_pos(1) = joint_end_pos_1;
  joint_end_pos(2) = joint_end_pos_2;
  joint_end_pos(3) = joint_end_pos_3;
  joint_end_pos(4) = joint_end_pos_4;
  joint_end_pos(5) = joint_end_pos_5;

  ROS_INFO("Start connect with UR5 to %s ...", robot_ip.c_str());

  if (!sim_robot) {
    try {
      RTDEReceiveInterface rtde_receive(robot_ip);
      std::vector<double> joint_positions = rtde_receive.getActualQ();

      if (joint_positions.size() != 6) {

        for (auto i = 0; i < joint_positions.size(); i++ ) {
          joint_start_pos(i) = joint_positions[i];
        }

      } else {
        throw "There should be 6 joints.";
      }

      ROS_INFO("Connect success!");

    } catch (const char* exception) {
      std::cerr << "Error: " << exception << '\n';

    } catch (...) {
        ROS_ERROR("I can't connect");
    }
  }

  UR5Trajopt example(env, plotter, debug, sim_robot, joint_start_pos, joint_end_pos);
  example.run();
}
