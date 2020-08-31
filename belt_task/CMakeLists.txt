cmake_minimum_required(VERSION 3.0.2)
project(belt_task)

set(CMAKE_CXX_STANDARD 11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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
  CATKIN_DEPENDS roscpp std_msgs
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
execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=alchemy --cflags OUTPUT_VARIABLE XENO_ALCHENMY_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=alchemy --ldflags OUTPUT_VARIABLE XENO_ALCHENMY_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

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


add_executable(belt_task ${SOURCE_FILES})
target_link_libraries(belt_task rtde sensor_filter sdu_math rt roscpp rosconsole roscpp_serialization boost_system rostime ${ROBWORK_LIBRARIES} ${yaml_cpp_LIBRARIES})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/belt_task_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
