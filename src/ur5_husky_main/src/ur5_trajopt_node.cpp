// ROS headers
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <termios.h>

#include <KinematicsUR5.hpp>
#include <TestIK.hpp>
#include <UR5Trajopt.hpp>
#include <UR5TrajoptResponce.hpp>
#include "robot_context/ur_rtde_interface.hpp"

#include <ur5_husky_main/SetStartJointState.h>
#include <ur5_husky_main/SetFinishJointState.h>
#include <ur5_husky_main/GetJointState.h>
#include <ur5_husky_main/RobotPlanTrajectory.h>
#include <ur5_husky_main/RobotExecuteTrajectory.h>
#include <ur5_husky_main/RobotRestart.h>
#include <ur5_husky_main/Box.h>
#include <ur5_husky_main/Mesh.h>
#include <ur5_husky_main/Freedrive.h>
#include <ur5_husky_main/IKSolver.h>
#include <ur5_husky_main/GetInfo.h>
#include <ur5_husky_main/GripperService.h>
#include <ur5_husky_main/InfoUI.h>
#include <ur5_husky_main/PoseList.h>
#include <ur5_husky_main/CalculateTrajectory.h>
#include <ur5_husky_main/GripperMoveRobot.h>
#include <ur5_husky_main/GetGripperState.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tesseract_environment/utils.h>
#include <tesseract_common/timer.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_msgs/EnvironmentState.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/EnvironmentCommand.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <ros/service.h>

#include <tesseract_geometry/impl/mesh_material.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>

#include <tesseract_collision/bullet/convex_hull_utils.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <tesseract_visualization/trajectory_player.h>

#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <ur_rtde/robotiq_gripper.h>

#include <actionlib/client/simple_action_client.h>
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_control/robotiq_gripper_client.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include <settings_custom_lib/SettingsCustomLib.hpp>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include "ColorInfo.hpp"

using namespace ur_rtde;
using namespace std;

using namespace ur5_husky_main;
using namespace tesseract_rosutils;

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

typedef robotiq_2f_gripper_control::RobotiqActionClient RobotiqActionClient;

SettingsCustomLibClass settingsConfig;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

Eigen::VectorXd joint_start_pos(6);
Eigen::VectorXd joint_end_pos(6);
std::vector<Eigen::VectorXd> joint_middle_pos_list;
bool robotPlanTrajectory = false;
bool robotExecuteTrajectory = false;
bool setRobotNotConnectErrorMes = false;
bool robotRestart = true;
bool freeDriveOn = false;
std::vector<ur5_husky_main::Gripper> gripperPoseList;
ur5_husky_main::Gripper gripperPosePrev;


ColorInfo getDefaultColor(std::string colorName) {
  if (colorName == "brown") {
    ColorInfo colorBrown{colorName, 0.83, 0.37, 0.2, 1.0};
    return colorBrown;
  } 
  ColorInfo colorDefault{colorName, 1.0, 1.0, 1.0, 1.0};
  return colorDefault;
}


tesseract_environment::Command::Ptr addBox(std::string link_name, std::string joint_name,
                                           float length, float width, float height,
                                           float pos_x, float pos_y, float pos_z,
                                           ColorInfo color) {

  auto colorBox = std::make_shared<tesseract_scene_graph::Material>(color.getName().c_str());
  colorBox->color = Eigen::Vector4d(color.getR(), color.getG(), color.getB(), color.getA());

  // Add sphere to environment
  Link link_sphere(link_name.c_str());

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(width, length, height);
  visual->material = colorBox;

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



tesseract_environment::Command::Ptr addMesh(std::string link_name,
                                            std::string joint_name,
                                            std::string mesh_name,
                                            Eigen::Vector3d scale,
                                            Eigen::Vector3d translation,
                                            Eigen::Vector3d rotation) {

  std::string mesh_path = "package://ur5_husky_main/meshes/objects/" + mesh_name;

  tesseract_common::ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  std::vector<tesseract_geometry::Mesh::Ptr> meshes =
    tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(
        locator->locateResource(mesh_path), scale, true, false, true);

  Link link_sphere(link_name.c_str());

  for (auto& mesh : meshes) {
    Visual::Ptr visual = std::make_shared<Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = mesh;
    link_sphere.visual.push_back(visual);

    Collision::Ptr collision = std::make_shared<Collision>();
    collision->origin = visual->origin;
    collision->geometry = makeConvexMesh(*mesh);
    link_sphere.collision.push_back(collision);
  }

  Joint joint_sphere(joint_name.c_str());
  joint_sphere.parent_link_name = "world";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
}


tesseract_environment::Command::Ptr renderMove(std::string link_name, std::string joint_name,
                                           float pos_x, float pos_y, float pos_z,
                                           float rotateX = 0, float rotateY = 0, float rotateZ = 0) {

  auto joint_limits =  std::make_shared<tesseract_scene_graph::JointLimits>();
  joint_limits->lower = 1.0;
  joint_limits->upper = 2.0;

  Eigen::AngleAxisd rotX(rotateX, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rotY(rotateY, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rotZ(rotateZ, Eigen::Vector3d::UnitZ());

  Joint joint_sphere(joint_name.c_str());
  joint_sphere.limits = joint_limits;
  joint_sphere.parent_link_name = "world";
  joint_sphere.child_link_name = link_name;
  joint_sphere.type = JointType::FIXED;

  joint_sphere.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
  joint_sphere.parent_to_joint_origin_transform.rotate(rotX);
  joint_sphere.parent_to_joint_origin_transform.rotate(rotY);
  joint_sphere.parent_to_joint_origin_transform.rotate(rotZ);

  return std::make_shared<tesseract_environment::MoveLinkCommand>(joint_sphere);
}


bool moveBox(ur5_husky_main::Box::Request &req,
             ur5_husky_main::Box::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::string joint_name = std::string(req.name) + "_joints";

  Command::Ptr box = renderMove(req.name, joint_name.c_str(), req.offsetX, req.offsetY, req.offsetZ, req.rotateX, req.rotateY, req.rotateZ);
  if (!env->applyCommand(box)) {
    res.result = "ERROR - create box";
    return false;
  }

  res.result = "Move Box end...";

  return true;
}


bool moveMesh(ur5_husky_main::Mesh::Request &req,
             ur5_husky_main::Mesh::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::string joint_name = std::string(req.name) + "_joints";

  Command::Ptr mesh = renderMove(req.name, joint_name.c_str(), req.offsetX, req.offsetY, req.offsetZ, req.rotateX, req.rotateY, req.rotateZ);
  if (!env->applyCommand(mesh)) {
    res.result = "ERROR - move mesh";
    return false;
  }

  res.result = "Move Mesh end...";

  return true;
}

tesseract_environment::Command::ConstPtr renderRemoveLink(std::string link_name) {
  return std::make_shared<tesseract_environment::RemoveLinkCommand>(link_name);
}

tesseract_environment::Command::ConstPtr renderRemoveJoint(std::string joints_name) {
  return std::make_shared<tesseract_environment::RemoveJointCommand>(joints_name);
}

tesseract_environment::Command::ConstPtr renderHideLink(std::string link_name) {
  return std::make_shared<tesseract_environment::ChangeLinkVisibilityCommand>(link_name, false);
}

bool removeBox(ur5_husky_main::Box::Request &req,
             ur5_husky_main::Box::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {
  Command::ConstPtr boxLink = renderRemoveLink(req.name);
  if (!env->applyCommand(boxLink)) {
    res.result = "ERROR - remove link box";
    return false;
  }

  res.result = "Remove Box end...";

  return true;
}


bool removeMesh(ur5_husky_main::Mesh::Request &req,
             ur5_husky_main::Mesh::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env,
             const std::shared_ptr<tesseract_monitoring::ROSEnvironmentMonitor> &monitor) {

  Command::ConstPtr meshLink = renderRemoveLink(req.name);
  if (!env->applyCommand(meshLink)) {
    res.result = "ERROR - remove link mesh";
    return false;
  }

  res.result = "Remove Mesh end...";

  return true;
}


void robotMove(std::vector<double> &path_pose) {
    try {

        // URRTDEInterface con(robot_ip);
        // auto rtde_control = con->rtdeControlConnect();

        RTDEControlInterface rtde_control(robot_ip);
        path_pose.push_back(ur_speed);
        path_pose.push_back(ur_acceleration);
        path_pose.push_back(ur_blend);

        std::vector<std::vector<double>> jointsPath;
        jointsPath.push_back(path_pose);
        rtde_control.moveJ(jointsPath);
        rtde_control.stopScript();
        rtde_control.disconnect();
    
    } catch (...) {
        if (!setRobotNotConnectErrorMes) {
          // 1 сообщения хватит
          setRobotNotConnectErrorMes = true;
          ROS_ERROR("I can't connect with UR5.");
        }
    }
}


bool updateStartJointValue(ur5_husky_main::SetStartJointState::Request &req,
                      ur5_husky_main::SetStartJointState::Response &res,
                      const std::shared_ptr<tesseract_environment::Environment> &env,
                      const ros::Publisher &joint_pub_state,
                      const std::vector<std::string> &joint_names,
                      const bool connect_robot,
                      ros::Rate &loop_rate,
                      ros::Publisher &gripperPub) {

    std::vector<double> position_vector;
    std::vector<double> velocity;
    std::vector<double> effort;

    int index = 0;
    for (int j = 0; j < joint_names.size(); j++) {
        for (int i = 0; i < req.name.size(); i++) {
            // нужны только избранные joints
            if (req.name[i] == joint_names[j]) {
                joint_start_pos(index) = req.position[i];
                velocity.push_back(req.velocity[i]);
                effort.push_back(req.effort[i]);

                index++;
            }
        }
    }

    position_vector.resize(joint_start_pos.size());
    Eigen::VectorXd::Map(&position_vector[0], joint_start_pos.size()) = joint_start_pos;

    std::cout << "gripper start: " << req.gripperPose[0].name << " " << req.gripperPose[0].angle << std::endl;
    gripperPosePrev.name = req.gripperPose[0].name;
    gripperPosePrev.angle = req.gripperPose[0].angle;

    if (connect_robot) {
        robotMove(position_vector);
        gripperPub.publish(req.gripperPose[0]);
    }

    env->setState(joint_names, joint_start_pos);

    ros::spinOnce();
    loop_rate.sleep();

    res.result = "End publish";
    return true;
}


bool updateFinishJointValue(ur5_husky_main::SetFinishJointState::Request &req,
                      ur5_husky_main::SetFinishJointState::Response &res,
                      const std::shared_ptr<tesseract_environment::Environment> &env,
                      const ros::Publisher &joint_pub_state,
                      const std::vector<std::string> &joint_names,
                      const bool connect_robot) {

    gripperPoseList.insert(gripperPoseList.begin(), std::begin(req.gripperPose), std::end(req.gripperPose));
    std::cout << "gripperPoseList: " << gripperPoseList.size() << std::endl;

    int index = 0;
    for (int j = 0; j < joint_names.size(); j++) {
        for (int i = 0; i < req.name.size(); i++) {
            // нужны только избранные joints
            if (req.name[i] == joint_names[j]) {
                joint_end_pos(index) = req.position[i];
                index++;
            }
        }
    }

    for (int i = 0; i < req.middlePose.size(); i++) {
      index = 0;
      Eigen::VectorXd joint_tmp(joint_names.size());

      for (int k = 0; k < joint_names.size(); k++) {
        for (int j = 0; j < req.middlePose[i].name.size(); j++) {
          if (req.middlePose[i].name[j] == joint_names[k]) {
            joint_tmp(index) = req.middlePose[i].position[j];
            index++;
          }
        }
      }
      joint_middle_pos_list.push_back(joint_tmp);
    }

    env->setState(joint_names, joint_end_pos);

    res.result = "Update Finish Position!";
    return true;
}


bool getJointValue(ur5_husky_main::GetJointState::Request &req,
                   ur5_husky_main::GetJointState::Response &res,
                   const std::vector<std::string> &joint_names,
                   const ros::Publisher &joint_pub_state,
                   const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::vector<double> joint_positions;

  if (req.from_robot) {
    try {
      RTDEReceiveInterface rtde_receive(robot_ip);
      ROS_INFO("Connect success!");
      std::vector<double> joint_positions = rtde_receive.getActualQ();

      for (auto i = 0; i < joint_positions.size(); i++) {
        joint_start_pos(i) = joint_positions[i];
      }

      env->setState(joint_names, joint_start_pos);

    } catch (...) {
      ROS_ERROR("Can`t connect with UR5!");
      env->setState(joint_names, joint_start_pos);
    }
  }

  joint_positions.resize(joint_start_pos.size());
  Eigen::VectorXd::Map(&joint_positions[0], joint_start_pos.size()) = joint_start_pos;

  // Это надо перенести отсюда 
  // не выставляем здесь значения!!!
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names;
  joint_state_msg.position = joint_positions;
  joint_pub_state.publish(joint_state_msg);

  res.name = joint_names;
  res.position = joint_positions;

  return true;
}


bool calculateRobotTrajectory(ur5_husky_main::CalculateTrajectory::Request &req, 
                              ur5_husky_main::CalculateTrajectory::Response &res,
                              const std::shared_ptr<tesseract_environment::Environment> &env,
                              const ROSPlottingPtr &plotter,
                              const std::vector<std::string> &joint_names,
                              const ros::Publisher &messagePosePub,
                              ros::Rate &loop_rate) {

  std::vector<double> start = req.startPose.position;
  std::vector<double> finish = req.finishPose.position;

  Eigen::VectorXd startPose = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(start.data(), start.size());
  Eigen::VectorXd finishPose = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(finish.data(), finish.size());

  std::vector<Eigen::VectorXd> middlePos;
  middlePos.empty();

  UR5Trajopt calculate(env, plotter, joint_names, startPose, finishPose, true, middlePos);
  UR5TrajoptResponce responce = calculate.run();

  tesseract_common::JointTrajectory trajectoryTrajOpt = responce.getTrajectory();

  std::vector<ur5_husky_main::Pose> trajectory;

  TrajectoryPlayer player;
  player.setTrajectory(trajectoryTrajOpt);

  for (int i = 0; i < trajectoryTrajOpt.states.size(); i++) {

    std::vector<double> position;

    tesseract_common::JointState j_state = player.getByIndex(i);
    position.resize(j_state.position.size());
    Eigen::VectorXd::Map(&position[0], j_state.position.size()) = j_state.position;

    ur5_husky_main::Pose msg;
    msg.name = j_state.joint_names;
    msg.position = position;
    trajectory.push_back(msg);
  }

  res.success = responce.isSuccessful();
  res.message = responce.getMessage();
  res.trajectory = trajectory;

  ur5_husky_main::PoseList poses;
  poses.pose_list = trajectory;

  messagePosePub.publish(poses);
  ros::spinOnce();
  loop_rate.sleep();

  return true;
}


bool robotPlanTrajectoryMethod(ur5_husky_main::RobotPlanTrajectory::Request &req, ur5_husky_main::RobotPlanTrajectory::Response &res) {

   robotPlanTrajectory = true;
   res.result = "Plan Trajectory";
   res.success = true;
   return true;
}

bool robotExecuteTrajectoryMethod(ur5_husky_main::RobotExecuteTrajectory::Request &req, ur5_husky_main::RobotExecuteTrajectory::Response &res) {
  robotExecuteTrajectory = true;
  res.result = "Execute Trajectory";
  res.success = true;
  return true;
}

bool robotRestartMethod(ur5_husky_main::RobotRestart::Request &req, ur5_husky_main::RobotRestart::Response &res) {
  robotRestart = true;

  // Откатить начальное значения
  robotPlanTrajectory = false;
  robotExecuteTrajectory = false;
  setRobotNotConnectErrorMes = false;
  res.result = "Robot restart Trajectory";
  return true;
}


bool createBox(ur5_husky_main::Box::Request &req,
              ur5_husky_main::Box::Response &res,
              const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::string joint_name = std::string(req.name) + "_joints";

  ColorInfo color{req.color.name, req.color.r, req.color.g, req.color.b, req.color.a};

  Command::Ptr box = addBox(req.name, joint_name.c_str(), req.length, req.width, req.height, req.x, req.y, req.z, color);
  if (!env->applyCommand(box)) {
    res.result = "ERROR - create box";
    return false;
  }

  // Сдвинуть box
  Command::Ptr move = renderMove(req.name, joint_name.c_str(), req.offsetX, req.offsetY, req.offsetZ, req.rotateX, req.rotateY, req.rotateZ);
  if (!env->applyCommand(move)) {
    res.result = "ERROR - move new box";
    return false;
  }

  res.result = "Create box success";
  return true;
}


bool createMesh(ur5_husky_main::Mesh::Request &req,
              ur5_husky_main::Mesh::Response &res,
              const std::shared_ptr<tesseract_environment::Environment> &env) {

  std::string joint_name = std::string(req.name) + "_joints";

  Command::Ptr mesh = addMesh(req.name, joint_name.c_str(), req.fileName,
                      Eigen::Vector3d(req.scale, req.scale, req.scale), 
                      Eigen::Vector3d(req.x, req.y, req.z), 
                      Eigen::Vector3d(req.rotateX, req.rotateY, req.rotateZ));

  // Создать меш
  if (!env->applyCommand(mesh)) {
    res.result = "ERROR - create mesh";
    return false;
  }

  // Сдвинуть меш
  Command::Ptr move = renderMove(req.name, joint_name.c_str(), req.offsetX, req.offsetY, req.offsetZ, req.rotateX, req.rotateY, req.rotateZ);
  if (!env->applyCommand(move)) {
    res.result = "ERROR - move new mesh";
    return false;
  }

  res.result = "Create mesh success";
  return true;
}


void freedriveControl(ros::Rate &loop_rate) {

  try {
    RTDEControlInterface rtde_control(robot_ip);
    RTDEReceiveInterface rtde_receive(robot_ip);
    DashboardClient dash_board(robot_ip);

    dash_board.connect();

    if (rtde_control.isConnected()) {

        rtde_control.teachMode();

        // Если значения ниже - 7 7 2, то freedrive включен
        std::cout << "Robot mode: " << rtde_receive.getRobotMode() << "\n";
        std::cout << "Robot status: " << rtde_receive.getRobotStatus() << "\n";
        std::cout << "RuntimeState: " << rtde_receive.getRuntimeState() << "\n";
        std::cout << dash_board.polyscopeVersion() << "\n";

        ROS_INFO("Freedrive on");

        while(ros::ok()) {
          ros::spinOnce();
          loop_rate.sleep();
          if (!freeDriveOn) {
            break;
          }
        }

        rtde_control.endTeachMode();
        std::cout << "Robot status: " << rtde_receive.getRobotStatus() << "\n";

        ROS_INFO("Freedrive off");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    rtde_control.stopScript();
    rtde_control.disconnect();
    dash_board.stop();
    dash_board.disconnect();
    std::cout<<"stop script..." <<std::endl;

  } catch(...) {
    ROS_ERROR(" Connect error with UR5 for Freedrive");
  }
}


bool freedriveEnable(ur5_husky_main::Freedrive::Request &req,
                    ur5_husky_main::Freedrive::Response &res) {

  std::string str = req.on ? "true" : "false";
  std::cout << "Freedrive comand: " <<  str << std::endl;
  freeDriveOn = req.on;

  std::string mess = "Freedrive command: " + str + ". Freedrive state changed! ";
  res.result = mess.c_str();

  return true;
}


bool getInfo(ur5_husky_main::GetInfo::Request &req,
                   ur5_husky_main::GetInfo::Response &res,
                   const std::shared_ptr<tesseract_environment::Environment> &env,
                   const std::vector<std::string> &joint_names,
                   ros::Rate &loop_rate) {

  TestIK test(robot_ip, ur_speed, ur_acceleration, ur_blend, req.debug);
  // try {
    DashboardClient dash_board(robot_ip);
    dash_board.connect();
    std::cout << "--- Модель робота ---" << dash_board.getRobotModel() << std::endl;
    // getSerialNumber() function is not supported on the dashboard server for PolyScope versions less than 5.6.0
    // std::cout << "--- Серийный номер робота ---" << dash_board.getSerialNumber() << std::endl;
    std::cout << "--- Программа робота: ---" << dash_board.getLoadedProgram() << std::endl;

    dash_board.stop();
    dash_board.disconnect();
  // } catch(...) {
  //   ROS_ERROR(" Connect error with UR5 for connect dash_board");
  // }

  if (req.fk || req.ik) {
    if (req.fk) {
      // Получить данные из библиотеки ur_rtde
      try {

        RTDEControlInterface rtde_control(robot_ip);
        if (rtde_control.isConnected()) {
          std::vector<double> fk = rtde_control.getForwardKinematics();
          if (req.fk) {
            std::cout << "Прямая кинематика (ur_rtde): ";
            for (int i = 0; i < fk.size(); i++) {
              std::cout << fk[i] << " ";
            }
            std::cout << std::endl;              
          }

          if (req.ik) {
              std::vector<double> ik = rtde_control.getInverseKinematics(fk);
              std::cout << "Обратная кинематика (ur_rtde): ";
              for (int i = 0; i < ik.size(); i++) {
                std::cout << ik[i] << " ";
              }
              std::cout << std::endl;
          }

          rtde_control.stopScript();
          rtde_control.disconnect();

        }
      } catch(...) {
        ROS_ERROR(" Connect error with UR5 for get info");
      }

      test.getForwardKinematics(joint_start_pos);
    }

    if (req.ik) {
      test.ikSolverCheck(loop_rate, joint_start_pos);
    }
  }

  res.result = "Info got successfully";
  return true;
}


bool gripperMove(ur5_husky_main::GripperService::Request &req,
                 ur5_husky_main::GripperService::Response &res,
                 ros::ServiceClient &gripperMoveRobotService, 
                 ur5_husky_main::GripperMoveRobot &srv) {

  srv.request.name = "";
  srv.request.speed = 0.10;
  srv.request.angle = req.angle;

  if (gripperMoveRobotService.call(srv)) {
    ROS_INFO("Move gripper send ang=%f", req.angle);
  } else {
    ROS_ERROR("Can not call service 'gripperMoveRobotService'");
    return false;
  }

  return true;
}


bool getGripperState(ur5_husky_main::GetGripperState::Request &req,
                 ur5_husky_main::GetGripperState::Response &res,
                 ros::ServiceClient &gsService, 
                 ur5_husky_main::GetGripperState &gripperStateSrv) {


  if (gsService.call(gripperStateSrv)) {
    res.angle = gripperStateSrv.response.angle;
  } else {
    ROS_ERROR("Can not call service 'gsService'");
    return false;
  }
  return true;
}


void gripperCallback(const ur5_husky_main::Gripper::ConstPtr &msg, ros::Publisher &gripperPub) {
  gripperPub.publish(msg);
}


void sendMessageToUI(std::string message, ros::Publisher &messageUIPub, ros::Rate &loop_rate, double time = 0.0) {
  ur5_husky_main::InfoUI msg;
  msg.code = message;
  msg.time = time;
  ROS_INFO("Sent message to UI: %s", msg.code.c_str());
  messageUIPub.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();
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
  bool ui_control = false;
  std::string script = "";

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
  pnh.param("ui_control", ui_control, ui_control);
  pnh.param("script", script, script);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ros::Publisher joint_pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1000);
  ros::Publisher joint_pub_state = pnh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  settingsConfig.update();

  auto env = std::make_shared<tesseract_environment::Environment>();

  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator)) {
    exit(1);
  }

  if (!ui_control) {
    // Создать стол
    Command::Ptr table = addBox("table", "joint_table_attached", table_length, table_width, table_height, table_pos_x, table_pos_y, table_pos_z, getDefaultColor("brown"));
    if (!env->applyCommand(table)) {
      return false;
    }

    // Создать коробку
    Command::Ptr box = addBox("box", "joint_box_attached", box_length, box_width, box_height, box_pos_x, box_pos_y, box_pos_z, getDefaultColor(""));
    if (!env->applyCommand(box)) {
      return false;
    }

    // Создать стол из mesh
    Command::Ptr table_mesh = addMesh("table", "table-js", "table-noise-2.obj", Eigen::Vector3d(0.015, 0.015, 0.015), 
      Eigen::Vector3d(1.7, 0.0, -0.14869), Eigen::Vector3d(0.0, 0.0, 0.0));
    if (!env->applyCommand(table_mesh)) {
      return false;
    }

    // Создать горелку из mesh
    Command::Ptr burner_mesh = addMesh("burner", "burner-js", "burner.obj", Eigen::Vector3d(0.03, 0.03, 0.03), 
      Eigen::Vector3d(1.0, 0.0, 0.57), Eigen::Vector3d(0.0, 0.0, 0.0));
    if (!env->applyCommand(burner_mesh)) {
      return false;
    }
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

  ros::Publisher messagePub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher messageUIPub = nh.advertise<ur5_husky_main::InfoUI>("chatter_ui", 1000);
  ros::Publisher messagePosePub = nh.advertise<ur5_husky_main::PoseList>("trajopt_pose", 1000);
  std_msgs::String msg;

  ros::Publisher gripperPub = nh.advertise<ur5_husky_main::Gripper>("gripper_state", 1000);

  ros::ServiceServer setStartJointsService = nh.advertiseService<ur5_husky_main::SetStartJointState::Request, ur5_husky_main::SetStartJointState::Response>
                      ("set_joint_start_value", boost::bind(updateStartJointValue, _1, _2, env, joint_pub_state, joint_names, connect_robot, loop_rate, gripperPub));

  ros::ServiceServer setFinishJointsService = nh.advertiseService<ur5_husky_main::SetFinishJointState::Request, ur5_husky_main::SetFinishJointState::Response>
                      ("set_joint_finish_value", boost::bind(updateFinishJointValue, _1, _2, env, joint_pub_state, joint_names, connect_robot));

  ros::ServiceServer getJointsService = nh.advertiseService<ur5_husky_main::GetJointState::Request, ur5_husky_main::GetJointState::Response>
                      ("get_joint_value", boost::bind(getJointValue, _1, _2, joint_names, joint_pub_state, env));

  ros::ServiceServer robotCalculatePlanService = nh.advertiseService<ur5_husky_main::CalculateTrajectory::Request, ur5_husky_main::CalculateTrajectory::Response>
                      ("calculate_robot_rajectory", boost::bind(calculateRobotTrajectory, _1, _2, env, plotter, joint_names, messagePosePub, loop_rate));

  ros::ServiceServer robotPlanService = nh.advertiseService<ur5_husky_main::RobotPlanTrajectory::Request, ur5_husky_main::RobotPlanTrajectory::Response>
                      ("robot_plan_trajectory", boost::bind(robotPlanTrajectoryMethod, _1, _2));

  ros::ServiceServer robotExecuteService = nh.advertiseService<ur5_husky_main::RobotExecuteTrajectory::Request, ur5_husky_main::RobotExecuteTrajectory::Response>
                      ("robot_execute_trajectory", boost::bind(robotExecuteTrajectoryMethod, _1, _2));

  ros::ServiceServer robotRestartService = nh.advertiseService<ur5_husky_main::RobotRestart::Request, ur5_husky_main::RobotRestart::Response>
                      ("robot_restart", boost::bind(robotRestartMethod, _1, _2));

  ros::ServiceServer createBoxService = nh.advertiseService<ur5_husky_main::Box::Request, ur5_husky_main::Box::Response>
                      ("create_box", boost::bind(createBox, _1, _2, env));

  ros::ServiceServer createMeshService = nh.advertiseService<ur5_husky_main::Mesh::Request, ur5_husky_main::Mesh::Response>
                      ("create_mesh", boost::bind(createMesh, _1, _2, env));

  ros::ServiceServer moveBoxService = nh.advertiseService<ur5_husky_main::Box::Request, ur5_husky_main::Box::Response>
                      ("move_box", boost::bind(moveBox, _1, _2, env));

  ros::ServiceServer moveMeshService = nh.advertiseService<ur5_husky_main::Mesh::Request, ur5_husky_main::Mesh::Response>
                      ("move_mesh", boost::bind(moveMesh, _1, _2, env));

  ros::ServiceServer removeBoxService = nh.advertiseService<ur5_husky_main::Box::Request, ur5_husky_main::Box::Response>
                      ("remove_box", boost::bind(removeBox, _1, _2, env));

  ros::ServiceServer removeMeshService = nh.advertiseService<ur5_husky_main::Mesh::Request, ur5_husky_main::Mesh::Response>
                      ("remove_mesh", boost::bind(removeMesh, _1, _2, env, monitor));

  ros::ServiceServer freedriveService = nh.advertiseService<ur5_husky_main::Freedrive::Request, ur5_husky_main::Freedrive::Response>
                      ("freedrive_change", boost::bind(freedriveEnable, _1, _2));

  ros::ServiceServer getInfoService = nh.advertiseService<ur5_husky_main::GetInfo::Request, ur5_husky_main::GetInfo::Response>
                      ("get_info_robot", boost::bind(getInfo, _1, _2, env, joint_names, loop_rate));

  ros::ServiceClient gripperMoveRobotService = nh.serviceClient<ur5_husky_main::GripperMoveRobot>("gripper_move_robot");
  ur5_husky_main::GripperMoveRobot srv;

  ros::ServiceServer gripperService = nh.advertiseService<ur5_husky_main::GripperService::Request, ur5_husky_main::GripperService::Response>
                      ("gripper_move", boost::bind(gripperMove, _1, _2, gripperMoveRobotService, srv));


  ros::ServiceClient gsService = nh.serviceClient<ur5_husky_main::GetGripperState>("gripper_state_robot");
  ur5_husky_main::GetGripperState gripperStateSrv;

  ros::ServiceServer gripperStateService = nh.advertiseService<ur5_husky_main::GetGripperState::Request, ur5_husky_main::GetGripperState::Response>
                      ("get_gripper_state", boost::bind(getGripperState, _1, _2, gsService, gripperStateSrv));                    

  ros::Subscriber sub = nh.subscribe<ur5_husky_main::Gripper>("gripper_state", 10, boost::bind(gripperCallback, _1, gripperPub));

  if (debug) {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  }


  while(ros::ok()) {

    bool goToStart = false;
    robotRestart = true;

    // Ждем команды для старта
    if (ui_control) {
      std::cout << "Waiting for the command to start ... \n";
      while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (robotRestart) {
          break;
        }
      }
    } else {
      plotter->waitForInput("Hit Enter for start.");
    }

    robotRestart = false;
    plotter->clear();

    // Ждем команды на планирование траектории
    if (ui_control && !robotPlanTrajectory) {
      std::cout << "Waiting for the command to plan the trajectory... \n";
      while(ros::ok()) {

        // До начала планирования можно подключить freedrive
        if (freeDriveOn) {
          msg.data = "freedrive_on";
          ROS_INFO("Sent message to UI: %s", msg.data.c_str());
          messagePub.publish(msg);

          freedriveControl(loop_rate);

          msg.data = "freedrive_off";
          ROS_INFO("Sent message to UI: %s", msg.data.c_str());
          messagePub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
        if (robotPlanTrajectory) {
          break;
        }

        if (robotRestart) {
          goToStart = true;
          break;
        }
      }

      if (goToStart) {
        continue;
      }
    } else if (!ui_control) {
      plotter->waitForInput("Hit Enter after move robot to start position.");
    }



    //////////////////////////////////////////////////
    //////////////////////////////////////////////////
    // Detach the simulated attach from the world and attach to the end effector
    tesseract_environment::Commands cmds;
    Joint joint_attach("joint_attach");
    joint_attach.parent_link_name = "ur5_tool0";
    joint_attach.child_link_name = "attach";

    if (script == "open_lock") {
      joint_attach.child_link_name = "locker_box_1";
    }

    joint_attach.type = JointType::FIXED;
    joint_attach.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    joint_attach.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(x_pos_correct, y_pos_correct, z_pos_correct);
    Eigen::AngleAxisd rotX(x_orient_correct, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotY(y_orient_correct, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotZ(z_orient_correct, Eigen::Vector3d::UnitZ());
    joint_attach.parent_to_joint_origin_transform.rotate(rotX);
    joint_attach.parent_to_joint_origin_transform.rotate(rotY);
    joint_attach.parent_to_joint_origin_transform.rotate(rotZ);

    cmds.push_back(std::make_shared<tesseract_environment::MoveLinkCommand>(joint_attach));
    tesseract_common::AllowedCollisionMatrix add_ac;
    // add_ac.addAllowedCollision("cup", "ur5_tool0", "Never");
    cmds.push_back(std::make_shared<tesseract_environment::ModifyAllowedCollisionsCommand>(
        add_ac, tesseract_environment::ModifyAllowedCollisionsType::ADD));
    env->applyCommands(cmds);
    ////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////
    //
    //    Проверить, необходимо ли менять положение гриппера
    //
    /////////////////////////////////////////////////////////////
    std::vector<Eigen::VectorXd> joint_middle_pos_list_tmp; // Промежуточные позы, которые будем отправлять на робота

    std::vector<tesseract_common::JointTrajectory> trajectoryList;
    std::vector<ur5_husky_main::Gripper> gripperStates;

    bool changeGripperState = false;
    double time = 0.0;

    std::cout << "Gripper: " << std::endl;

    for (int i = 0; i < joint_middle_pos_list.size(); i++) {
      if (i < gripperPoseList.size()) {

        std::cout << gripperPoseList[i].name << " " << gripperPoseList[i].angle << std::endl;

        joint_middle_pos_list_tmp.push_back(joint_middle_pos_list[i]);
        if (gripperPoseList[i].angle != gripperPosePrev.angle) {
          changeGripperState = true;
          gripperStates.push_back(gripperPoseList[i]);

          UR5Trajopt calculate(env, plotter, joint_names, joint_start_pos, joint_end_pos, ui_control, joint_middle_pos_list_tmp);
          UR5TrajoptResponce responce = calculate.run();
          bool success = responce.isSuccessful();

          // TODO подумать, что тут делать в случае ошибки (при расчете сложной траектории из множества кусков)
          if (!success) {
            ROS_ERROR("Planning (PLAN №%d) ended with errors!", i+1);
            sendMessageToUI("error_trajopt", messageUIPub, loop_rate, responce.getTime());
            robotPlanTrajectory = false;
            robotExecuteTrajectory = false;
            setRobotNotConnectErrorMes = false;
            continue;

          } else {
            tesseract_common::JointTrajectory trajectory = responce.getTrajectory();
            trajectoryList.push_back(trajectory);
            time += responce.getTime();
          }

          joint_middle_pos_list_tmp.clear();
        }

        gripperPosePrev = gripperPoseList[i];
      } else {
        joint_middle_pos_list_tmp.push_back(joint_middle_pos_list[i]);
      }
    }


    // Если в принципе не было смены состояния гриппера, то тоже считаем план
    if (!changeGripperState) {
      UR5Trajopt calculate(env, plotter, joint_names, joint_start_pos, joint_end_pos, ui_control, joint_middle_pos_list);
      UR5TrajoptResponce responce = calculate.run();
      bool success = responce.isSuccessful();

      if (!success) {
        ROS_ERROR("Planning ended with errors!");
        sendMessageToUI("error_trajopt", messageUIPub, loop_rate, responce.getTime());
        robotPlanTrajectory = false;
        robotExecuteTrajectory = false;
        setRobotNotConnectErrorMes = false;
        continue;
      }

      tesseract_common::JointTrajectory trajectory = responce.getTrajectory();
      trajectoryList.push_back(trajectory);
      time = responce.getTime();
    }

    sendMessageToUI("plan_finish", messageUIPub, loop_rate, time);

    /////////////////////////////////////////////////
    //
    //   Выполнение траектории в симуляторе rViz и на роботе
    //
    /////////////////////////////////////////////////

    // TODO Проверка флага на подключение connect_robot

    char input_simbol = 'n';

    if (!ui_control) {
      std::cout << "Execute Trajectory on UR5? y/n \n";
      std::cin >> input_simbol;
    }

    // Ждем команды на выполнение траектории
    if (ui_control && !robotExecuteTrajectory) {
      std::cout << "Waiting for the command to execute the trajectory... \n";
      while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (robotExecuteTrajectory) {
          break;
        }

        if (robotRestart) {
          goToStart = true;
          break;
        }
      }
    }

    if (goToStart) {
      continue;
    }
    
    if (robotExecuteTrajectory || input_simbol == 'y') {
      std::cout << "Executing... \n";

      for (int j = 0; j < trajectoryList.size(); j++) {

          tesseract_common::JointTrajectory trajectory = trajectoryList[j];

          TrajectoryPlayer player;
          player.setTrajectory(trajectory);

          std::vector<tesseract_common::JointState> j_states;
              
          ROS_INFO("Added intermediate joints: ");

          // Список доступных положений робота
          std::vector<std::vector<double>> jointsPath;
          std::vector<double> path_pose;

          for (int i = 0; i < trajectory.states.size(); i++) {

            tesseract_common::JointState j_state = player.getByIndex(i);
            path_pose.resize(j_state.position.size());
            Eigen::VectorXd::Map(&path_pose[0], j_state.position.size()) = j_state.position;
            path_pose.push_back(ur_speed);
            path_pose.push_back(ur_acceleration);
            path_pose.push_back(ur_blend);
            jointsPath.push_back(path_pose);

            ROS_INFO("%d point of traectory: ", i+1);

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
          }

          std::cout << "jointsPath: " << jointsPath.size() << std::endl;

          // Обновить состояние до последней позиции
          position_vector.resize(joint_end_pos.size());
          Eigen::VectorXd::Map(&position_vector[0], joint_end_pos.size()) = joint_end_pos;

          path_pose.resize(position_vector.size());
          Eigen::VectorXd::Map(&path_pose[0], joint_end_pos.size()) = joint_end_pos;
          path_pose.push_back(ur_speed);
          path_pose.push_back(ur_acceleration);
          path_pose.push_back(ur_blend);
          jointsPath.push_back(path_pose);

          // Отправить на робота
          try {
            RTDEControlInterface rtde_control(robot_ip);
            rtde_control.moveJ(jointsPath);
            rtde_control.stopScript();

          } catch (...) {
            ROS_ERROR("Can`t connect with UR5.");
          }

          // Поменять состояние гриппера
          std::cout << "gripperStates = " << gripperStates.size() << std::endl;
          std::cout << "j = " << j << std::endl;
          if (j < gripperStates.size()) {
            std::cout << "PUB GRIPPER STATE = " << gripperStates[j].name << " " << gripperStates[j].angle << std::endl;
            gripperPub.publish(gripperStates[j]);
          }

          // Установить состояние для tesseract
          env->setState(joint_names, joint_end_pos);

          // Сообщение для отправки конечного состояния для обновления в rViz
          sensor_msgs::JointState joint_state_msg;
          joint_state_msg.name = joint_names;
          joint_state_msg.position = position_vector;
          joint_state_msg.velocity = velocity_default; // скорость
          joint_state_msg.effort = effort_default; // усилие
          joint_pub_state.publish(joint_state_msg);
      }

      // Установить итоговое состояние гриппера после выполнения траектории
      gripperPub.publish(gripperPoseList[gripperPoseList.size()-1]);

    } else {
      std::cout << "The trajectory in the simulator will not be executed. \n";
    }

    sendMessageToUI("execute_finish", messageUIPub, loop_rate);

    robotPlanTrajectory = false;
    robotExecuteTrajectory = false;
    setRobotNotConnectErrorMes = false;
    joint_middle_pos_list_tmp.clear();
    gripperPoseList.clear();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
