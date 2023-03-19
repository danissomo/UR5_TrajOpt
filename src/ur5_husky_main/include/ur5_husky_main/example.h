#ifndef UR5_HUSKY_MAIN_EXAMPLE_H
#define UR5_HUSKY_MAIN_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_visualization/visualization.h>

namespace ur5_husky_main
{
/**
 * @brief The Example base class
 *
 * It provides a generic interface for all examples as a library which then
 * can easily be integrated as unit tests so breaking changes are caught.
 * Also it provides a few utility functions for checking rviz environment and
 * updating the rviz environment.
 */
class Example
{
public:
  Example(tesseract_environment::Environment::Ptr env, tesseract_visualization::Visualization::Ptr plotter = nullptr)
    : env_(std::move(env)), plotter_(std::move(plotter))
  {
  }

  virtual ~Example() = default;
  Example(const Example&) = default;
  Example& operator=(const Example&) = default;
  Example(Example&&) = default;
  Example& operator=(Example&&) = default;

  virtual tesseract_common::JointTrajectory run() = 0;

protected:
  /** @brief Tesseract Manager Class (Required) */
  tesseract_environment::Environment::Ptr env_;
  /** @brief Tesseract Visualization Class (Optional)*/
  tesseract_visualization::Visualization::Ptr plotter_;
};

}
#endif  // UR5_HUSKY_MAIN_EXAMPLE_H
