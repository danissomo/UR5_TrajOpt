#include "ur5_trajopt.hpp"

#include <ros/ros.h>

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
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
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <tesseract_visualization/trajectory_player.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include <settings_custom_lib/AddSettingsCustomLib.hpp>

using namespace tesseract_rosutils;

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;



UR5Trajopt::UR5Trajopt (tesseract_environment::Environment::Ptr env,
                        ROSPlottingPtr plotter,
                        std::vector<std::string> joint_names,
                        Eigen::VectorXd joint_start_pos,
                        Eigen::VectorXd joint_end_pos,
                        bool ui_control) {
  env_ = env;
  plotter_ = plotter;
  joint_names_ = joint_names;
  joint_start_pos_ = joint_start_pos;
  joint_end_pos_ = joint_end_pos;
  ui_control_ = ui_control;
}

tesseract_common::JointTrajectory UR5Trajopt::run() {
  
  // промежуточное положение робота
  Eigen::VectorXd joint_middle_pos(6);
  joint_middle_pos(0) = joint_middle_pos_0;
  joint_middle_pos(1) = joint_middle_pos_1;
  joint_middle_pos(2) = joint_middle_pos_2;
  joint_middle_pos(3) = joint_middle_pos_3;
  joint_middle_pos(4) = joint_middle_pos_4;
  joint_middle_pos(5) = joint_middle_pos_5;

  Eigen::VectorXd joint_middle_pos2(6);
  joint_middle_pos2(0) = joint_middle2_pos_0;
  joint_middle_pos2(1) = joint_middle2_pos_1;
  joint_middle_pos2(2) = joint_middle2_pos_2;
  joint_middle_pos2(3) = joint_middle2_pos_3;
  joint_middle_pos2(4) = joint_middle2_pos_4;
  joint_middle_pos2(5) = joint_middle2_pos_5;

   // Solve Trajectory
  CONSOLE_BRIDGE_logInform("UR5 trajopt plan");

  // Create Program
  CompositeInstruction program("UR5", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "ur5_tool0"));

  // Start and End Joint Position for the program
  StateWaypointPoly wp0{ StateWaypoint(joint_names_, joint_start_pos_) };
  StateWaypointPoly wp01{ StateWaypoint(joint_names_, joint_middle_pos) };
  StateWaypointPoly wp02{ StateWaypoint(joint_names_, joint_middle_pos2) };
  StateWaypointPoly wp1{ StateWaypoint(joint_names_, joint_end_pos_) };

  MoveInstruction start_instruction(wp0, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Plan freespace from start
  // Assign a linear motion so cartesian is defined as the target

  MoveInstruction plan_f01(wp01, MoveInstructionType::FREESPACE, "UR5");
  plan_f01.setDescription("freespace_plan");

  MoveInstruction plan_f02(wp02, MoveInstructionType::FREESPACE, "UR5");
  plan_f02.setDescription("freespace_plan");

  MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "UR5");
  plan_f0.setDescription("freespace_plan");

  // Add Instructions to program
  program.appendMoveInstruction(plan_f01);
  program.appendMoveInstruction(plan_f02);
  program.appendMoveInstruction(plan_f0);

  // Print Diagnostics
  program.print("Program: ");

  // Create Executor
  auto executor = std::make_unique<TaskflowTaskComposerExecutor>(5);

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();

  // Тип коллизии задается в настройках
  // trajopt::CollisionEvaluatorType collisionCostConfigType;
  // trajopt::CollisionEvaluatorType collisionConstraintConfigType;
  // if (collision_cost_config_type == "SINGLE_TIMESTEP") {
  //     collisionCostConfigType = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  // } else if (collision_cost_config_type == "CAST_CONTINUOUS") {
  //     collisionCostConfigType = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  // } else if (collision_constraint_config_type == "SINGLE_TIMESTEP") {
  //     collisionConstraintConfigType = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  // } else if (collision_constraint_config_type == "CAST_CONTINUOUS") {
  //     collisionConstraintConfigType = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  // } else { // DISCRETE_CONTINUOUS - вариант по умолчанию
  //     collisionCostConfigType = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  //     collisionConstraintConfigType = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  // }

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  // composite_profile->longest_valid_segment_length = 0.05;
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
  profiles->addProfile<TrajOptCompositeProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UR5", composite_profile);

  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
  plan_profile->cartesian_coeff(0) = 0;
  plan_profile->cartesian_coeff(1) = 0;
  plan_profile->cartesian_coeff(2) = 0;

  // auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
  // trajopt_solver_profile->opt_info.max_iter = 100;

  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UR5", plan_profile);
  // profiles->addProfile<TrajOptSolverProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UR5", trajopt_solver_profile);

  // Create Task Input Data
  TaskComposerDataStorage input_data;
  input_data.setData("input_program", program);

  // Create Task Composer Problem
  TaskComposerProblem problem(env_, input_data);

  // Задержка, чтобы показать сцену, потом строить траекторию
  if (!ui_control_ && plotter_ != nullptr && plotter_->isConnected()) {
    plotter_->waitForInput("Hit Enter to solve for trajectory.");
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
  if (plotter_ != nullptr && plotter_->isConnected()) {
    if (!ui_control_) {
      plotter_->waitForInput();
    }
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    auto state_solver = env_->getStateSolver();
    auto scene_state = env_->getState();

    plotter_->plotMarker(ToolpathMarker(toolpath));
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  return trajectory;
}