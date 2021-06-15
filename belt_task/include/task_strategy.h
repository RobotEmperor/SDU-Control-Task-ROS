/*
 * task_strategy.h
 *
 *  Created on: Oct 20, 2020
 *      Author: yik
 */
#ifndef SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_TASK_STRATEGY_H_
#define SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_TASK_STRATEGY_H_

#define DEGREE2RADIAN M_PI/180.0
#define RADIAN2DEGREE  180.0/M_PI

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
#include <rw/math.hpp>

//log
#include "log.h"

//ros msg
#include "geometry_msgs/PoseArray.h"

using namespace rw::math;

class TaskStrategy
{
public:
  TaskStrategy();
  TaskStrategy(std::string robot_name);
  ~TaskStrategy();

  void initialize(const std::string &path);
  void assign_parts(std::string master, std::string slave);
  //recieve motion planning
  void input_paths(geometry_msgs::PoseArray paths_);
  void input_b_paths(geometry_msgs::PoseArray paths_);
  double calculate_velocity(double first_point_, double second_point_, double interval_time_);
  void calculate_init_final_velocity(int point_number_);
  double calculate_next_velocity(double first_vel_, double second_vel_);

  void initialize_reference_frame();
  bool tasks();

  //belt task
  void estimation_of_belt_position();
  void decision_of_function();

  // m-s relationship
  void master_robot();
  void slave_robot();

  void check_phases();

  //input data
  void set_is_moving_check(bool check);
  void set_robot_current_tf(Transform3D<> tf_current_pose);

  //belt task
  void set_parts_data(std::vector<double> first_part, std::vector<double> second_part);
  void set_type(std::string type);

  //output data
  double get_gripper_move_values();
  double get_controller_smooth_gain_change();
  Transform3D<> get_current_reference_frame_();
  Transform3D<> get_a_part_frame_();
  Transform3D<> get_b_part_frame_();
  std::vector<double> get_current_way_points_();
  std::vector<double> get_current_way_init_vel_points_();
  std::vector<double> get_current_way_final_vel_points_();
  std::vector<double> get_current_desired_force_vector_();
  bool get_finish_task_check();

  std::string get_type();

private:
  int pre_phases_;
  int phases_;
  int all_phases_;

  int master_way_points_numbers_;
  int slave_way_points_numbers_;
  int desired_force_values_numbers_;

  std::string robot_name_;
  std::string file_path_;

  //command
  std::string type_;
  std::string pre_type_;

  //initial pose
  std::vector<double> initial_pose_vector_; //(6);
  std::vector<double> initial_pose_vector_rpy_; //(6);
  rw::math::Transform3D<> tf_initial_pose_;


  //master robot way points
  std::map<int, std::vector<double>> master_way_points_;
  std::map<int, std::vector<double>> master_way_init_vel_points_;
  std::map<int, std::vector<double>> master_way_final_vel_points_;

  //slave robot way points
  std::map<int, std::vector<double>> slave_way_points_;

  //roboust force values
  std::map<int, std::vector<double>> desired_force_values_;

  //current way points
  Transform3D<> current_reference_frame_;
  std::vector<double> current_way_points_;
  std::vector<double> current_way_init_vel_points_;
  std::vector<double> current_way_final_vel_points_;
  std::vector<double> current_desired_force_vector_;

  //algorithm
  //pulley direction
  rw::math::Transform3D<> tf_base_to_tcp_;
  rw::math::Transform3D<> tf_base_to_tcp_rotated_;
  rw::math::Transform3D<> tf_base_to_start_;
  rw::math::Transform3D<> tf_base_to_end_;

  rw::math::Transform3D<> tf_tcp_to_start_;
  rw::math::Transform3D<> tf_tcp_to_end_;

  rw::math::Transform3D<> tf_tcp_to_direction_;
  rw::math::Transform3D<> tf_tcp_to_rotate_;

  //estimation of belt pose
  rw::math::Vector3D<> insert_belt_way_points_;
  rw::math::Vector3D<> desired_groove_position_;

  rw::math::Transform3D<> tf_a_parts;
  rw::math::Transform3D<> tf_b_parts;

  // input
  // robot is moving check
  Transform3D<> tf_current_pose_;
  std::vector<double> first_part_;
  std::vector<double> second_part_;
  bool is_moving_check_;

  //output
  //grippers
  double gripper_move_values_;
  double smooth_gain_change_time_;

  int finish_tasks_;
  int sub_tasks_;
};





#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TASK_STRATEGY_H_ */
