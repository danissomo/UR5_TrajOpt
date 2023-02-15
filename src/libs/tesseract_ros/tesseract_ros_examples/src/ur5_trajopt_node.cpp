#include <tesseract_examples/ur5_trajopt.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>

using namespace ur_rtde;

using namespace tesseract_examples;
using namespace tesseract_rosutils;

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
  joint_start_pos(0) = 0.0;
  joint_start_pos(1) = -0.06;
  joint_start_pos(2) = -2.72;
  joint_start_pos(3) = -0.34;
  joint_start_pos(4) = 0.0;
  joint_start_pos(5) = 0.0;

  ROS_INFO("Start connect with UR5...");

  if (!sim_robot) {
    try {
      RTDEReceiveInterface rtde_receive("127.0.0.1");
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

  UR5Trajopt example(env, plotter, debug, sim_robot, joint_start_pos);
  example.run();
}
