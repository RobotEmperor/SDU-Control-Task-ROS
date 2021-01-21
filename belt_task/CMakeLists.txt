cmake_minimum_required(VERSION 3.0.2)
project(belt_task)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
  sdu_math
)
add_action_files(
  DIRECTORY action
  FILES belt_task_action.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

find_package(Eigen3 REQUIRED)

set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")

find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sdu_math roscpp actionlib actionlib_msgs std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS}) 
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

set( SOURCE_FILES 
  src/data_logging.cpp
  src/task_motion.cpp
  src/ros_node.cpp
  src/main.cpp
  src/tool_estimation.cpp
  src/task_robot.cpp
  src/task_strategy.cpp
  src/object_estimation.cpp
)

add_executable(belt_task ${SOURCE_FILES})
target_include_directories(belt_task PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ROBWORK_INCLUDE_DIR}
  ${yaml_cpp_INCLUDE_DIRS}
)
#target_link_libraries(belt_task  pthread sdu_math actionlib rtde rt roscpp rosconsole roscpp_serialization boost_system rostime ${ROBWORK_LIBRARIES} ${yaml_cpp_LIBRARIES})
target_link_libraries(belt_task pthread ${catkin_LIBRARIES} rtde ${ROBWORK_LIBRARIES} ${yaml_cpp_LIBRARIES})
add_dependencies(belt_task belt_task_generate_messages_cpp)
