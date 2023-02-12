#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/ur5_trajopt.h>
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

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

namespace tesseract_examples
{
UR5Trajopt::UR5Trajopt(tesseract_environment::Environment::Ptr env,
                                         tesseract_visualization::Visualization::Ptr plotter,
                                         bool debug,
                                         bool sim_robot)
  : Example(std::move(env), std::move(plotter)), debug_(debug), sim_robot_(sim_robot)
{
}

bool UR5Trajopt::run() {

  if (plotter_ != nullptr) {
    plotter_->waitForConnection();
  }

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.emplace_back("shoulder_pan_joint");
  joint_names.emplace_back("shoulder_lift_joint");
  joint_names.emplace_back("elbow_joint");
  joint_names.emplace_back("wrist_1_joint");
  joint_names.emplace_back("wrist_2_joint");
  joint_names.emplace_back("wrist_3_joint");

  Eigen::VectorXd joint_start_pos(6);

  if (sim_robot_) {
    joint_start_pos(0) = 0.0;
    joint_start_pos(1) = -0.06;
    joint_start_pos(2) = -2.72;
    joint_start_pos(3) = -0.34;
    joint_start_pos(4) = 0.0;
    joint_start_pos(5) = 0.0;

  } else {
    // тут надо получить данные с робота

  }

  

  Eigen::VectorXd joint_end_pos(6);
  joint_end_pos(0) = 0.0;
  joint_end_pos(1) = -0.68;
  joint_end_pos(2) = -1.75;
  joint_end_pos(3) = -0.68;
  joint_end_pos(4) = 0.0;
  joint_end_pos(5) = 0.0;

  env_->setState(joint_names, joint_start_pos);

  if (debug_) {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  }

  // Solve Trajectory
  CONSOLE_BRIDGE_logInform("UR5 trajopt plan");

  // Create Program
  CompositeInstruction program(
      "UR5", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

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
  profiles->addProfile<TrajOptCompositeProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UR5", composite_profile);

  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
  plan_profile->cartesian_coeff(0) = 0;
  plan_profile->cartesian_coeff(1) = 0;
  plan_profile->cartesian_coeff(2) = 0;

  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "UR5", plan_profile);

  // Create Task Input Data
  TaskComposerDataStorage input_data;
  input_data.setData("input_program", program);

  // Create Task Composer Problem
  TaskComposerProblem problem(env_, input_data);

  if (plotter_ != nullptr && plotter_->isConnected()) {
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

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected()) {
    plotter_->waitForInput();
    auto ci = input.data_storage.getData("output_program").as<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();

    plotter_->plotMarker(ToolpathMarker(toolpath));
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  CONSOLE_BRIDGE_logInform("Final trajectory is collision free");


















  return input.isSuccessful();
}
}  // namespace tesseract_examples
