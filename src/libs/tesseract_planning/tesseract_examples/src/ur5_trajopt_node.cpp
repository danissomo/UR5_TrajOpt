#include <tesseract_examples/ur5_trajopt.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_examples;
using namespace tesseract_common;
using namespace tesseract_environment;

int main(int /*argc*/, char** /*argv*/) {
  auto locator = std::make_shared<TesseractSupportResourceLocator>();
  tesseract_common::fs::path urdf_path =
      locator->locateResource("package://ur5_single_arm_tufts/urdf/ur5_single_arm.urdf.xacro")->getFilePath();
  tesseract_common::fs::path srdf_path =
      locator->locateResource("package://ur5_single_arm_manipulation/urdf/ur5_trajopt.srdf")->getFilePath();
  auto env = std::make_shared<Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
    exit(1);

  UR5Trajopt example(env, nullptr);
  if (!example.run())
    exit(1); 
}