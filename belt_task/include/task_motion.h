/*
 * task_motion.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_TASK_MOTION_H_
#define SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_TASK_MOTION_H_

#include <Eigen/Dense>
// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>

//yaml
#include <yaml-cpp/yaml.h>

//sdu_math
#include "sdu_math/kinematics.h"
#include "sdu_math/end_point_to_rad_cal.h"
#include <rw/math.hpp>

//log
#include "log.h"

using namespace rw::math;

class TaskMotion
{

public:
  TaskMotion();
  ~TaskMotion();
  //setup
  void initialize(double control_time_);
  void motion_to_desired_pose(Transform3D<> reference_frame, double x, double y, double z,RPY<> tcp_rpy_, double time);
  void add_desired_vel(std::vector<double> init_vel_, std::vector<double> final_vel_);

  void generate_fifth_order_trajectory();

  bool is_moving_check();

  void set_initial_pose(double x, double y, double z, double axis_x, double axis_y, double axis_z);
  void set_current_pose_eaa(double x, double y, double z, double axis_x, double axis_y, double axis_z);
  void set_current_pose_rpy(double x, double y, double z, double yaw, double pitch, double roll);

  void stop_motion();

  std::vector<double> get_current_pose();

private:
  //robot's ee position
  std::vector<double> current_pose_vector_;
  std::vector<double> current_force_torque_vector_;

  std::shared_ptr<EndEffectorTraj> robot_traj_;
  Eigen::Matrix<double, 6, 8> desired_pose_matrix_;

  rw::math::Transform3D<> tf_base_to_bearing_start_;
  rw::math::Transform3D<> tf_base_to_bearing_end_;

  rw::math::Transform3D<> tf_tcp_desired_pose_;
  rw::math::Transform3D<> tf_current_pose_;
  rw::math::Transform3D<> tf_desired_pose_;

  bool flag_;
};
#endif /* TASK_MOTION_H_ */

