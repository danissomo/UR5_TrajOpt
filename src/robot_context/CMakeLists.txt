cmake_minimum_required(VERSION 3.0.2)
project(robot_context)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(ur_rtde REQUIRED PATHS "/workspace/src/ur_rtde-v1.5.0/build/ur_rtde")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ur_rtde_interface.cpp
  src/robot_env_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
   PRIVATE ur_rtde::rtde
   ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)