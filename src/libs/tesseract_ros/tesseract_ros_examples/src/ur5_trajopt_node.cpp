// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


#include <tesseract_examples/ur5_trajopt.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tesseract_environment/utils.h>
#include <tesseract_common/timer.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_task_composer/task_composer_problem.h>
#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_node_names.h>
#include <tesseract_task_composer/nodes/trajopt_motion_pipeline_task.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

#include <settings_custom_lib/SettingsCustomLib.hpp>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

using namespace ur_rtde;

using namespace tesseract_examples;
using namespace tesseract_rosutils;

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

SettingsCustomLibClass settingsConfig;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";



// Convert to joint trajectory
// tesseract_common::JointTrajectory getJointTrajectory(const std::vector<std::string>& joint_names,
//                                                      const tesseract_common::TrajArray& current_trajectory) {
//   tesseract_common::JointTrajectory joint_traj;
//   joint_traj.reserve(static_cast<std::size_t>(current_trajectory.rows()));
//   double total_time = 0;
//   for (long i = 0; i < current_trajectory.rows(); ++i)
//   {
//     tesseract_common::JointState js(joint_names, current_trajectory.row(i));
//     js.time = total_time;
//     joint_traj.push_back(js);
//     total_time += 0.1;
//   }
//   return joint_traj;
// }


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5_trajopt_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::Rate loop_rate(0.2);

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

  ros::Publisher test_pub = nh.advertise<trajectory_msgs::JointTrajectory>("traj", 10);

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


  std::vector<std::string> joint_names;
  joint_names.emplace_back("shoulder_pan_joint");
  joint_names.emplace_back("shoulder_lift_joint");
  joint_names.emplace_back("elbow_joint");
  joint_names.emplace_back("wrist_1_joint");
  joint_names.emplace_back("wrist_2_joint");
  joint_names.emplace_back("wrist_3_joint");

  Eigen::VectorXd joint_start_pos(6);
  joint_start_pos(0) = joint_start_pos_0;
  joint_start_pos(1) = joint_start_pos_1;
  joint_start_pos(2) = joint_start_pos_2;
  joint_start_pos(3) = joint_start_pos_3;
  joint_start_pos(4) = joint_start_pos_4;
  joint_start_pos(5) = joint_start_pos_5;

  env->setState(joint_names, joint_start_pos);

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

  if (debug) {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  }

  // Solve Trajectory
  CONSOLE_BRIDGE_logInform("UR5 trajopt plan");

  // Create Program
  CompositeInstruction program(
      "UR5-1", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start and End Joint Position for the program
  StateWaypointPoly wp0{ StateWaypoint(joint_names, joint_start_pos) };
  StateWaypointPoly wp1{ StateWaypoint(joint_names, joint_end_pos) };

  MoveInstruction start_instruction(wp0, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Plan freespace from start
  // Assign a linear motion so cartesian is defined as the target
  MoveInstruction plan_f0(wp1, MoveInstructionType::LINEAR, "UR5");
  plan_f0.setDescription("freespace_plan");

  // Add Instructions to program
  program.appendMoveInstruction(plan_f0);

  // Print Diagnostics
  program.print("Program: ");

  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>(5);

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  composite_profile->collision_cost_config.safety_margin = 0.01;
  composite_profile->collision_cost_config.safety_margin_buffer = 0.01;
  composite_profile->collision_cost_config.coeff = 1;
  composite_profile->collision_constraint_config.enabled = true;
  composite_profile->collision_constraint_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  composite_profile->collision_constraint_config.safety_margin = 0.01;
  composite_profile->collision_constraint_config.safety_margin_buffer = 0.01;
  composite_profile->collision_constraint_config.coeff = 1;
  composite_profile->smooth_velocities = true;
  composite_profile->smooth_accelerations = false;
  composite_profile->smooth_jerks = false;
  composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
  profiles->addProfile<TrajOptCompositeProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UR5-1", composite_profile);

  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  // plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
  // plan_profile->cartesian_coeff(0) = 0;
  // plan_profile->cartesian_coeff(1) = 0;
  // plan_profile->cartesian_coeff(2) = 0;

  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UR5-2", plan_profile);

  // Create Task Input Data
  TaskComposerDataStorage input_data;
  input_data.setData("input_program", program);

  // Create Task Composer Problem
  TaskComposerProblem problem(env, input_data);

  //if (plotter != nullptr && plotter->isConnected()) {
    //plotter->waitForInput("Hit Enter to solve for trajectory.");  // Тут загружается робот
  //}

  // Solve process plan
  tesseract_common::Timer stopwatch;
  stopwatch.start();
  TaskComposerInput input(problem, profiles);
  TrajOptMotionPipelineTask task("input_program", "output_program");
  TaskComposerFuture::UPtr future = executor->run(task, input);
  future->wait();
  stopwatch.stop();
  CONSOLE_BRIDGE_logInform("Planning took %f seconds.", stopwatch.elapsedSeconds());


  auto ci = input.data_storage.getData("output_program").as<CompositeInstruction>();
  tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
  std::vector<tesseract_planning::InstructionPoly> points = ci.getInstructions();
  std::vector<StateWaypointPoly> data_trajectory;

  // Plot Process Trajectory
  if (plotter != nullptr && plotter->isConnected()) {
    plotter->waitForInput();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env);
    auto state_solver = env->getStateSolver();

    plotter->plotMarker(ToolpathMarker(toolpath));
    plotter->plotTrajectory(trajectory, *state_solver);
  }


  for (int i = 0; i < points.size(); i++) {
    StateWaypointPoly point = points[i].as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
    data_trajectory.push_back(point);
    ROS_INFO("%d. Added intermediate joints: ", i+1);
    point.print();
  }

  std::cout << "Execute Trajectory on rViz or other simelator? y/n \n";
  char input_simbol = 'n';
  std::cin >> input_simbol;
  if (input_simbol == 'y') {
    std::cout << "Executing... \n";

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execution_client("follow_joint_trajectory",
                                                                                              true);

    control_msgs::FollowJointTrajectoryGoal trajectory_action;
    trajectory_msgs::JointTrajectory traj_msg;
    ros::Duration t(0.25);
    traj_msg = toMsg(trajectory, env->getState());
    trajectory_action.trajectory = traj_msg;


    execution_client.sendGoal(trajectory_action);
    execution_client.waitForResult(ros::Duration(20.0));

    if (execution_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      std::cout << "Action succeeded! \n";
    } else {
      std::cout << "Action failed \n";
    }

  } else {
    std::cout << "The trajectory in the simulator will not be executed. \n";
  }

  input_simbol = 'n';
  std::cout << "Execute Trajectory on hardware? y/n \n";
  std::cin >> input_simbol;
  if (input_simbol == 'y') {
    std::cout << "Executing... \n";

    //TODO 

  }


  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  CONSOLE_BRIDGE_logInform("Final trajectory is collision free");

  return 0;

}
