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

#include <robot_context/ur_rtde_interface.hpp>

#include <ur5_husky_scene/Box.h>
#include <ur5_husky_scene/Mesh.h>
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

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <settings_custom_lib/SettingsCustomLib.hpp>

#include "ColorInfo.hpp"

using namespace std;

using namespace tesseract_rosutils;

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
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

  std::string mesh_path = "package://ur5_husky_scene/meshes/objects/" + mesh_name;

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


bool moveBox(ur5_husky_scene::Box::Request &req,
             ur5_husky_scene::Box::Response &res,
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


bool moveMesh(ur5_husky_scene::Mesh::Request &req,
             ur5_husky_scene::Mesh::Response &res,
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

bool removeBox(ur5_husky_scene::Box::Request &req,
             ur5_husky_scene::Box::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {
  Command::ConstPtr boxLink = renderRemoveLink(req.name);
  if (!env->applyCommand(boxLink)) {
    res.result = "ERROR - remove link box";
    return false;
  }

  res.result = "Remove Box end...";

  return true;
}


bool removeMesh(ur5_husky_scene::Mesh::Request &req,
             ur5_husky_scene::Mesh::Response &res,
             const std::shared_ptr<tesseract_environment::Environment> &env) {

  Command::ConstPtr meshLink = renderRemoveLink(req.name);
  if (!env->applyCommand(meshLink)) {
    res.result = "ERROR - remove link mesh";
    return false;
  }

  res.result = "Remove Mesh end...";

  return true;
}


bool createBox(ur5_husky_scene::Box::Request &req,
              ur5_husky_scene::Box::Response &res,
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


bool createMesh(ur5_husky_scene::Mesh::Request &req,
              ur5_husky_scene::Mesh::Response &res,
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


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5_scene_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::Rate loop_rate(delay_loop_rate);

  bool ui_control = true;

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  auto env = std::make_shared<tesseract_environment::Environment>();

  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator)) {
    exit(1);
  }

  ros::ServiceServer createBoxService = nh.advertiseService<ur5_husky_scene::Box::Request, ur5_husky_scene::Box::Response>
                      ("create_box", boost::bind(createBox, _1, _2, env));

  ros::ServiceServer createMeshService = nh.advertiseService<ur5_husky_scene::Mesh::Request, ur5_husky_scene::Mesh::Response>
                      ("create_mesh", boost::bind(createMesh, _1, _2, env));

  ros::ServiceServer moveBoxService = nh.advertiseService<ur5_husky_scene::Box::Request, ur5_husky_scene::Box::Response>
                      ("move_box", boost::bind(moveBox, _1, _2, env));

  ros::ServiceServer moveMeshService = nh.advertiseService<ur5_husky_scene::Mesh::Request, ur5_husky_scene::Mesh::Response>
                      ("move_mesh", boost::bind(moveMesh, _1, _2, env));

  ros::ServiceServer removeBoxService = nh.advertiseService<ur5_husky_scene::Box::Request, ur5_husky_scene::Box::Response>
                      ("remove_box", boost::bind(removeBox, _1, _2, env));

  ros::ServiceServer removeMeshService = nh.advertiseService<ur5_husky_scene::Mesh::Request, ur5_husky_scene::Mesh::Response>
                      ("remove_mesh", boost::bind(removeMesh, _1, _2, env));

  if (!ui_control) {
    // TODO Тут падает
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

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
