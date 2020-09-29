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
  void initialize(double control_time_, std::string load_path_);
  void initialize_reference_frame(std::vector<double> reference_frame);
  void load_task_motion(std::string path_, std::string motion_);


  //a variety of motions all of things are with respect to TCP frame
  void make_belt_robust(double radious);
  void close_to_pulleys(double x,double y,double depth,RPY<> tcp_rpy_);
  void insert_belt_into_pulley(bool contact_, double change_x, double change_y, double change_z,RPY<> tcp_rpy_);
  void up_motion(bool contact_, double x, double y, double z,RPY<> tcp_rpy_);
  void finish_1(bool contact_, double x, double y, double z,RPY<> tcp_rpy_);
  void finish_2(bool contact_, double x, double y, double z,RPY<> tcp_rpy_);
  void rotate(double theta_);

  void motion_to_desired_pose(bool contact_, double x, double y, double z,RPY<> tcp_rpy_, double time);


  void estimation_of_belt_position(rw::math::Vector3D<> desired_groove_position);
  void insert_into_groove(RPY<> tcp_rpy_);

  void check_phases();
  void generate_trajectory();

  void clear_task_motion();
  void clear_phase();

  void set_initial_pose(double x, double y, double z, double axis_x, double axis_y, double axis_z);
  void set_current_pose_eaa(double x, double y, double z, double axis_x, double axis_y, double axis_z);
  void set_all_phases_(int all_phases_);

  void load_data_initialize();
  void load_data_tcp_motion();

  void stop_motion();

  std::vector<double> get_set_point_base();
  std::vector<double> get_current_pose();
  std::vector<double> get_desired_force_torque();
  std::vector<double> get_initial_ee_position();
  int get_phases_();

private:
  int number_of_point;
  int all_point;
  int init_all_point;
  int init_belt_task_all_point;
  int tcp_all_point;

  int current_point;
  bool check_change;
  bool task_done;
  bool base_frame_;

  double path_angle_;
  double path_y_;
  double change_path_x_;
  double change_path_y_;
  double change_path_z_;
  int phases_;
  int all_phases_;
  int pre_phases_;

  //initial condition
  //robot's ee position
  std::vector<double> initial_robot_ee_position;
  std::vector<double> bigger_pulley_bearing_position;
  std::vector<double> smaller_pulley_bearing_position;
  std::vector<double> task_initial_position;
  std::vector<double> radious_;
  std::vector<double> set_point_;

  //modified robot position (in relative to base)
  std::map<int, std::vector<double>> motion_start_time_vector;
  std::map<int, std::vector<double>> motion_task_pose_vector;
  std::map<int, std::vector<double>> motion_task_init_vel_vector;
  std::map<int, std::vector<double>> motion_task_final_vel_vector;

  //robot initial position (in relative to base)
  std::map<int, std::vector<double>> init_motion_start_time_vector;
  std::map<int, std::vector<double>> init_motion_task_pose_vector;
  std::map<int, std::vector<double>> init_motion_task_init_vel_vector;
  std::map<int, std::vector<double>> init_motion_task_final_vel_vector;

  std::vector<double> current_pose_vector;
  std::vector<double> current_force_torque_vector;

  std::shared_ptr<EndEffectorTraj> robot_traj;
  Eigen::MatrixXd desired_pose_matrix;

  Transform3D<> tf_base_to_bearing_;
  Transform3D<> tf_bearing_to_init_;
  Transform3D<> tf_base_to_init_task_;
  Transform3D<> tf_static_frame_;

  Transform3D<> tf_tcp_desired_pose_;
  Transform3D<> tf_current_pose_;
  Transform3D<> tf_desired_pose_;

  // tf for belt task
  Transform3D<> temp_;
  Transform3D<> tf_master_robot_to_ref_belt_;
  Transform3D<> tf_master_robot_to_desired_groove_;
  Transform3D<> tf_base_to_slave_robot_;
  Transform3D<> tf_base_to_bearing_slave_robot_;

  Transform3D<> tf_bearing_ref_belt_;
  Transform3D<> tf_bearing_desired_groove_;
  Transform3D<> tf_bearing_to_master_robot_;
  Transform3D<> tf_bearing_to_slave_robot_;

  rw::math::Vector3D<> ref_belt_;
  rw::math::Vector3D<> error_;
  rw::math::Vector3D<> modified_master_robot_;

  rw::math::Vector3D<> desired_groove_position_;

  rw::math::EAA<> initial_value;



  bool flag_;
};
#endif /* TASK_MOTION_H_ */

