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

using namespace rw::math;

class TaskStrategy
{

public:
  TaskStrategy();
  ~TaskStrategy();

  void initialize_frame(const std::string &path);
  void assign_parts(const std::string &path, std::string master, std::string slave);


  double get_gripper_move_values();

  // m-s relationship
  void master_robot();
  void slave_robot();

  void check_phases();

private:
  int pre_phases_;
  int phases_;
  int all_phases_;

  int master_way_points_numbers_;
  int slave_way_points_numbers_;


  //master robot way points
  std::map<int, std::vector<double>> master_way_points_;

  //slave robot way points
  std::map<int, std::vector<double>> slave_way_points_;

  //roboust force values
  std::map<int, std::vector<double>> tighten_force_values_;
  std::map<int, std::vector<double>> force_vector_;

  //current way points
  Transform3D<> current_reference_frame_;
  std::vector<double> current_way_points_;
  std::vector<double> current_desired_force_vector_;

  int tighten_force_values_numbers_;

  //grippers
  double gripper_move_values_;

  int sub_tasks_;

};





#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TASK_STRATEGY_H_ */
