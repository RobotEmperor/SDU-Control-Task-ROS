cmake_minimum_required(VERSION 3.0.2)
project(belt_task)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
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
#  LIBRARIES belt_task
  CATKIN_DEPENDS roscpp actionlib actionlib_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /opt/ros/melodic/include
  /usr/include/xenomai
  ${EIGEN3_INCLUDE_DIR}
  ${ROBWORK_INCLUDE_DIR}
  ${yaml_cpp_INCLUDE_DIRS}
  /home/yik/sdu_ws/include
)


#usr/xenomai/bin/
execute_process(COMMAND xeno-config --skin=alchemy --cflags OUTPUT_VARIABLE XENO_ALCHENMY_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND xeno-config --skin=alchemy --ldflags OUTPUT_VARIABLE XENO_ALCHENMY_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${XENO_ALCHENMY_CFLAGS} ${XENO_ALCHENMY_LDFLAGS}")

set( SOURCE_FILES 
  src/data_logging.cpp
  src/task_motion.cpp
  src/ros_node.cpp
  src/main.cpp
  src/tool_estimation.cpp
  src/task_robot.cpp
)

link_directories(${CMAKE_SOURCE_DIR}/lib
                 /home/yik/sdu_ws/lib
                 /opt/ros/melodic/lib
                 /usr/lib
                 )


add_executable(belt_task ${SOURCE_FILES} )
target_link_libraries(belt_task actionlib rtde sensor_filter sdu_math rt roscpp rosconsole roscpp_serialization boost_system rostime ${ROBWORK_LIBRARIES} ${yaml_cpp_LIBRARIES})
#add_dependencies(belt_task belt_task_generate_messages_cpp)
