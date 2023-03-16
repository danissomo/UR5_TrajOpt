#ifndef TESSERACT_EXAMPLES_UR5_TRAJOPT_H
#define TESSERACT_EXAMPLES_UR5_TRAJOPT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/example.h>

namespace tesseract_examples
{

class UR5Trajopt : public Example
{
public:
  UR5Trajopt(tesseract_environment::Environment::Ptr env,
                      tesseract_visualization::Visualization::Ptr plotter = nullptr,
                      bool debug = false,
                      bool sim_robot = true,
                      Eigen::VectorXd joint_start_pos = Eigen::VectorXd::Zero(6),
                      Eigen::VectorXd joint_end_pos = Eigen::VectorXd::Zero(6));
  ~UR5Trajopt() override = default;
  UR5Trajopt(const UR5Trajopt&) = default;
  UR5Trajopt& operator=(const UR5Trajopt&) = default;
  UR5Trajopt(UR5Trajopt&&) = default;
  UR5Trajopt& operator=(UR5Trajopt&&) = default;

  bool run() override final;

private:
  bool debug_;
  bool sim_robot_;
  Eigen::VectorXd joint_start_pos_;
  Eigen::VectorXd joint_end_pos_;

  static tesseract_environment::Command::Ptr addBox(std::string link_name, std::string joint_name, float length, float width, float height, float pos_x, float pos_y, float pos_z);
};

}

#endif