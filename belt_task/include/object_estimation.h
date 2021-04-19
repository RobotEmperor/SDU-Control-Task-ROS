/*
 * object_estimation.h
 *
 *  Created on: Jan 21, 2021
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_OBJECT_ESTIMATION_H_
#define SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_OBJECT_ESTIMATION_H_


#include <Eigen/Dense>
// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>

//yaml
#include <yaml-cpp/yaml.h>

//math
#include <rw/math.hpp>

using namespace rw::math;

class ObejectEstimation
{
public:
   ObejectEstimation();
  ~ObejectEstimation();
  //setup
  void initialize(int all_point_number,  const std::string &path);

  void model_estimation();

  void set_current_grab_poses(std::vector<std::vector<double>> current_grabs_poses);
  void set_initial_grab_poses(std::vector<std::vector<double>> initial_grabs_poses);

  void set_current_grab_poses(rw::math::Transform3D<> current_grabs_poses);
  void set_initial_grab_poses(rw::math::Transform3D<> initial_grabs_poses);

  void set_current_grab_vels(std::vector<std::vector<double>> current_grabs_vel);
  void set_initial_grab_vels(std::vector<std::vector<double>> initial_grabs_vel);

  std::vector<double> get_current_object_pose(int point_number);
  std::vector<double> get_current_object_force(int point_number);

  double get_current_object_force();

private:
  //robot's gripper position
  std::vector<std::vector<double>> current_grabs_pose_;
  std::vector<std::vector<double>> initial_grabs_pose_;

  rw::math::Transform3D<> current_grabs_tf_pose_;
  rw::math::Transform3D<> initial_grabs_tf_pose_;

  std::vector<std::vector<double>> current_grabs_vel_;
  std::vector<std::vector<double>> initial_grabs_vel_;

  //objects' position
  std::vector<std::vector<double>>  current_object_pose_vector_;
  std::vector<std::vector<double>>  current_object_force_torque_vector_;

  double spring_constant_k_;
  double damper_constant_c_;

  double objects_force_magnitude_;
  double initial_grab_point_magnitude_;
  double current_grab_point_magnitude_;

  //rw::math::Transform3D<> tf_base_to_bearing_start_;
};
#endif /* SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_OBJECT_ESTIMATION_H_ */
