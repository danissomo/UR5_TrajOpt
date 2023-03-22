// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


#include <ur5_husky_main/ur5_trajopt.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

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

#include <tesseract_visualization/trajectory_player.h>

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

using namespace ur5_husky_main;
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

Eigen::VectorXd joint_start_pos(6);



tesseract_environment::Command::Ptr addBox(std::string link_name, std::string joint_name,
                                           float length, float width, float height,
                                           float pos_x, float pos_y, float pos_z) {

  auto table = std::make_shared<tesseract_scene_graph::Material>("orange");
  table->color = Eigen::Vector4d(0.83, 0.37, 0.2, 1.0);

  auto box = std::make_shared<tesseract_scene_graph::Material>("white");
  box->color = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);

  // Add sphere to environment
  Link link_sphere(link_name.c_str());

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(width, length, height);

  if (link_name == "table") {
    visual->material = table;
  } else if (link_name == "box") {
    visual->material = box;
  }

  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere(joint_name.c_str());
  joint_sphere.parent_link_name = "world";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
}


void updateJointValue(const sensor_msgs::JointState::ConstPtr &msg,
                      const std::shared_ptr<tesseract_environment::Environment> &env,
                      const std::vector<std::string> &joint_names,
                      const bool connect_robot) {

    int index = 0;
    for (int j = 0; j < joint_names.size(); j++) {
        for (int i = 0; i < msg->name.size(); i++) {
            // нужны только избранные joints
            if (msg->name[i] == joint_names[j]) {
                joint_start_pos(index) = msg->position[i];
                index++;
            }
        }
    } 

    env->setState(joint_names, joint_start_pos);

    if (connect_robot) {
        std::vector<double> position_vector;
        position_vector.resize(joint_start_pos.size());
        Eigen::VectorXd::Map(&position_vector[0], joint_start_pos.size()) = joint_start_pos;

        try {
            RTDEControlInterface rtde_control(robot_ip);
            rtde_control.moveJ(position_vector);
            rtde_control.stopScript();
        } catch (...) {
            ROS_ERROR("I can't connect with UR5.");
        }
    }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5_trajopt_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::Rate loop_rate(delay_loop_rate);

  bool plotting = true;
  bool rviz = true;
  bool debug = false;
  bool connect_robot = false;

  // конфиги для робота
  double velocity = 0.5;
  double acceleration = 0.5;
  double dt = 1.0/500; // 2ms
  double lookahead_time = 0.1;
  double gain = 300;

  std::vector<double> velocity_default{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
  std::vector<double> effort_default{0.0, 0.0, 0.0, 0.0, 0.0, 0,0};
  std::vector<double> accelerations_default{0.1, 0.2, 0.3, 0.4, 0.5, 0,6};
  std::vector<double> position_vector;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("debug", debug, debug);
  pnh.param("connect_robot", connect_robot, connect_robot);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ros::Publisher joint_pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 10);
  ros::Publisher joint_pub_state = pnh.advertise<sensor_msgs::JointState>("/joint_states", 10);

  settingsConfig.update();

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator)) {
    exit(1);
  }

  // Создать стол
  Command::Ptr table = addBox("table", "joint_table_attached", table_length, table_width, table_height, table_pos_x, table_pos_y, table_pos_z);
  if (!env->applyCommand(table)) {
    return false;
  }

  // Создать коробку
  Command::Ptr box = addBox("box", "joint_box_attached", box_length, box_width, box_height, box_pos_x, box_pos_y, box_pos_z);
  if (!env->applyCommand(box)) {
    return false;
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
  joint_names.emplace_back("ur5_shoulder_pan_joint");
  joint_names.emplace_back("ur5_shoulder_lift_joint");
  joint_names.emplace_back("ur5_elbow_joint");
  joint_names.emplace_back("ur5_wrist_1_joint");
  joint_names.emplace_back("ur5_wrist_2_joint");
  joint_names.emplace_back("ur5_wrist_3_joint");

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

  if (connect_robot) { // Соединение с роботом (в симуляции или с реальным роботом)
    ROS_INFO("Start connect with UR5 to %s ...", robot_ip.c_str());
    try {
      RTDEReceiveInterface rtde_receive(robot_ip);
      ROS_INFO("Connect success!");
      std::vector<double> joint_positions = rtde_receive.getActualQ();

      if (joint_positions.size() == 6) {

        for (auto i = 0; i < joint_positions.size(); i++ ) {
          joint_start_pos(i) = joint_positions[i];
        }

      } else {
        throw "There should be 6 joints.";
      }

      env->setState(joint_names, joint_start_pos);

    } catch (const char* exception) {
      std::cerr << "Error: " << exception << '\n';
      env->setState(joint_names, joint_start_pos);

    } catch (...) {
        ROS_ERROR("I can't connect with UR5.");
        env->setState(joint_names, joint_start_pos);
    }

  } else {
      ROS_INFO("I work without connecting to the robot.");
      env->setState(joint_names, joint_start_pos);
  }

  // Установить начальное положение JointState
  position_vector.resize(joint_start_pos.size());
  Eigen::VectorXd::Map(&position_vector[0], joint_start_pos.size()) = joint_start_pos;
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names;
  joint_state_msg.position = position_vector;
  joint_state_msg.velocity = velocity_default;
  joint_state_msg.effort = effort_default;
  joint_pub_state.publish(joint_state_msg);

  // Подписчик на изменения joint state
  boost::function<void (const sensor_msgs::JointState::ConstPtr &msg)> func = 
                        boost::bind(updateJointValue, _1, env, joint_names, connect_robot);
  ros::Subscriber sub = pnh.subscribe("/joint_states", 10, func);

  if (debug) {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  }

  plotter->waitForInput("Hit Enter after move robot to start position.");

  // Solve Trajectory
  CONSOLE_BRIDGE_logInform("UR5 trajopt plan");

  // Create Program
  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "ur5_tool0"));

  // Start and End Joint Position for the program
  StateWaypointPoly wp0{ StateWaypoint(joint_names, joint_start_pos) };
  StateWaypointPoly wp1{ StateWaypoint(joint_names, joint_end_pos) };

  MoveInstruction start_instruction(wp0, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Plan freespace from start
  // Assign a linear motion so cartesian is defined as the target
  MoveInstruction plan_f0(wp1, MoveInstructionType::LINEAR, "DEFAULT");
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
  profiles->addProfile<TrajOptCompositeProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", composite_profile);

  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
  plan_profile->cartesian_coeff(0) = 0;
  plan_profile->cartesian_coeff(1) = 0;
  plan_profile->cartesian_coeff(2) = 0;

  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", plan_profile);

  // Create Task Input Data
  TaskComposerDataStorage input_data;
  input_data.setData("input_program", program);

  // Create Task Composer Problem
  TaskComposerProblem problem(env, input_data);

  // Задержка, чтобы показать сцену, потом строить траекторию
  if (plotter != nullptr && plotter->isConnected()) {
    plotter->waitForInput("Hit Enter to solve for trajectory.");
  }

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

  // Plot Process Trajectory
  if (plotter != nullptr && plotter->isConnected()) {
    plotter->waitForInput();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env);
    auto state_solver = env->getStateSolver();
    auto scene_state = env->getState();

    plotter->plotMarker(ToolpathMarker(toolpath));
    plotter->plotTrajectory(trajectory, *state_solver);
  }


  /////////////////////////////////////////////////
  //
  //   Выполнение траектории в симуляторе rViz
  //
  /////////////////////////////////////////////////

  // TODO Проверка флага на подключение connect_robot

  std::cout << "Execute Trajectory on UR5? y/n \n";
  char input_simbol = 'n';
  std::cin >> input_simbol;
  if (input_simbol == 'y') {
    std::cout << "Executing... \n";

    TrajectoryPlayer player;
    player.setTrajectory(trajectory);

    std::vector<tesseract_common::JointState> j_states;

    //////// Сообщение для отправки траектории
    // trajectory_msgs::JointTrajectory joint_traj_msg;
    // joint_traj_msg.header.stamp = ros::Time::now();
    // joint_traj_msg.header.frame_id = "0";
    // joint_traj_msg.joint_names = joint_names;
    // double time_from_start = 0.0;
    // double time_step = 0.2;
    ////////////////////////////////////

    
    ROS_INFO("Added intermediate joints: ");

    // Проверка связи с роботом или с ursim
    RTDEControlInterface rtde_control(robot_ip);


    for (int i = 0; i < points.size(); i++) {

      if (i == 0) {
        // loop_rate.sleep();
      }

      ROS_INFO("%d point of traectory: ", i+1);

      tesseract_common::JointState j_state = player.getByIndex(i);
      j_states.push_back(j_state);

      std::cout << "joint_names: ";
      for (const auto& name: j_state.joint_names) {
        std::cout << name.c_str() << ' ';
      }
      std::cout << std::endl;
      
      std::cout << "positions: " << j_state.position.transpose() << std::endl;
      std::cout << "velocity: " << j_state.velocity.transpose() << std::endl;
      std::cout << "acceleration: " << j_state.acceleration.transpose() << std::endl;
      std::cout << "effort: " << j_state.effort.transpose() << std::endl;
      std::cout << "====================" << std::endl;

      position_vector.resize(j_state.position.size());
      Eigen::VectorXd::Map(&position_vector[0], j_state.position.size()) = j_state.position;

      // Сообщение для отправки состояния
      sensor_msgs::JointState joint_state_msg;
      joint_state_msg.name = j_state.joint_names;
      joint_state_msg.position = position_vector;
      joint_state_msg.velocity = velocity_default; // скорость
      joint_state_msg.effort = effort_default; // усилие

      rtde_control.moveJ(position_vector);
      // rtde_control.stopScript();
      ROS_INFO("UR5 changed joints value");

      env->setState(joint_names, j_state.position);
      joint_pub_state.publish(joint_state_msg);
      // loop_rate.sleep();

      /////// Сообщения для отправки траектории
      // trajectory_msgs::JointTrajectoryPoint joint_points_msg;
      // joint_points_msg.positions = position_vector;
      // joint_points_msg.velocities = velocity;
      // joint_points_msg.accelerations = accelerations;
      // joint_points_msg.effort = effort;
      // time_from_start += time_step;
      // joint_points_msg.time_from_start = ros::Duration(time_from_start);
      // joint_traj_msg.points.push_back(joint_points_msg);
      /////////////////////////////////////////////////////
      
    }

    /////// Сообщения для отправки траектории
    //joint_pub_traj.publish(joint_traj_msg);
    /////////////////////////////////////////

    // Обновить состояние до последней позиции
    position_vector.resize(joint_end_pos.size());
    Eigen::VectorXd::Map(&position_vector[0], joint_end_pos.size()) = joint_end_pos;

    // Сообщение для отправки состояния
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name = joint_names;
    joint_state_msg.position = position_vector;
    joint_state_msg.velocity = velocity_default; // скорость
    joint_state_msg.effort = effort_default; // усилие

    rtde_control.moveJ(position_vector);

    env->setState(joint_names, joint_end_pos);
    joint_pub_state.publish(joint_state_msg);
    rtde_control.stopScript();

  } else {
    std::cout << "The trajectory in the simulator will not be executed. \n";
  }



  /////////////////////////////////////////////////
  //
  //      Выполнение траектории на роботе
  //
  /////////////////////////////////////////////////

  // input_simbol = 'n';
  // std::cout << "Execute Trajectory on hardware? y/n \n";
  // std::cin >> input_simbol;
  // if (input_simbol == 'y') {
  //   std::cout << "Executing... \n";

  //   actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execution_client("follow_joint_trajectory",
  //                                                                                             true);

  //   control_msgs::FollowJointTrajectoryGoal trajectory_action;
  //   trajectory_msgs::JointTrajectory traj_msg;
  //   ros::Duration t(0.25);
  //   traj_msg = toMsg(trajectory, env->getState());
  //   trajectory_action.trajectory = traj_msg;

  //   execution_client.sendGoal(trajectory_action);
  //   execution_client.waitForResult(ros::Duration(20.0));

  //   if (execution_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
  //     std::cout << "Action succeeded! \n";
  //   } else {
  //     std::cout << "Action failed \n";
  //   }
  // } else {
  //   std::cout << "You have selected \"Do not execute trajectory\" \n";
  // }

  // CONSOLE_BRIDGE_logInform("Final trajectory is collision free");

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
