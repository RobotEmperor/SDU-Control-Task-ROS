/*
 * task_strategy.cpp
 *
 *  Created on: Oct 20, 2020
 *      Author: yik
 */
#include "task_strategy.h"

TaskStrategy::TaskStrategy()
{
  pre_phases_ = 0;
  phases_ = 0;
  all_phases_ = 0;

  master_way_points_numbers_ = 0;
  slave_way_points_numbers_ = 0;

  tighten_force_values_numbers_ = 0;

  //grippers
  gripper_move_values_= 0;

  sub_tasks_ = 0;
}
TaskStrategy::~TaskStrategy()
{

}
void TaskStrategy::initialize_frame(const std::string &path)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); //

  }catch(const std::exception& e) //
  {
    std::cout << "Fail to load yaml file!" << std::endl;
    return;
  }
  YAML::Node part_position_node = doc["part_position"];
  YAML::Node desired_force_value_node = doc["desired_force_value"];

  std::vector<double> temp_points_;
  int temp_points_numbers_ = 0;

  temp_points_.clear();
  for (YAML::iterator it = desired_force_value_node.begin(); it != desired_force_value_node.end(); ++it)
  {
    tighten_force_values_numbers_ ++;
    temp_points_numbers_ = it->first.as<int>();

    for(int num = 0; num < 3; num++)
    {
      temp_points_.push_back(it->second[num].as<double>());
    }

    tighten_force_values_[temp_points_numbers_] = temp_points_;
    force_vector_[temp_points_numbers_] = temp_points_;

    temp_points_.clear();
  }
}
void TaskStrategy::assign_parts(const std::string &path, std::string master, std::string slave)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); //

  }catch(const std::exception& e) //
  {
    std::cout << "Fail to load yaml file! assign_pulley" << std::endl;
    return;
  }

  YAML::Node master_node = doc[master];
  YAML::Node slave_node = doc[slave];

  std::vector<double> temp_points_;
  int temp_points_numbers_ = 0;

  master_way_points_numbers_ = 0;

  for (YAML::iterator it = master_node.begin(); it != master_node.end(); ++it)
  {
    master_way_points_numbers_ ++;
    temp_points_numbers_ = it->first.as<int>();

    for(int num = 0; num < 3; num++)
    {
      temp_points_.push_back(it->second[num].as<double>());
    }
    for(int num = 3; num < 6; num++)
    {
      temp_points_.push_back(it->second[num].as<double>()*DEGREE2RADIAN);
    }
    temp_points_.push_back(it->second[6].as<double>());

    master_way_points_[temp_points_numbers_] = temp_points_;

    temp_points_.clear();

    //cout << robot_name_ << " " << temp_points_numbers_ <<"::"<< master_way_points_[temp_points_numbers_]  << endl;
  }

  temp_points_.clear();

  slave_way_points_numbers_ = 0;

  for (YAML::iterator it = slave_node.begin(); it != slave_node.end(); ++it)
  {
    slave_way_points_numbers_ ++;
    temp_points_numbers_ = it->first.as<int>();

    for(int num = 0; num < 3; num++)
    {
      temp_points_.push_back(it->second[num].as<double>());
    }
    for(int num = 3; num < 6; num++)
    {
      temp_points_.push_back(it->second[num].as<double>()*DEGREE2RADIAN);
    }

    temp_points_.push_back(it->second[6].as<double>());

    slave_way_points_[temp_points_numbers_] = temp_points_;

    temp_points_.clear();

    //cout << robot_name_ << " " <<  temp_points_numbers_ <<"::"<< slave_way_points_[temp_points_numbers_]  << endl;
  }
}
void TaskStrategy::initialize_reference_frame(std::vector<double> temp_reference_frame_start, std::vector<double> temp_reference_frame_end, std::string command)
{
  rw::math::Transform3D<> tf_base_to_tcp_;
  rw::math::Transform3D<> tf_base_to_tcp_rotated_;
  rw::math::Transform3D<> tf_base_to_start_;
  rw::math::Transform3D<> tf_base_to_end_;

  rw::math::Transform3D<> tf_tcp_to_start_;
  rw::math::Transform3D<> tf_tcp_to_end_;

  rw::math::Transform3D<> tf_tcp_to_direction_;
  rw::math::Transform3D<> tf_tcp_to_rotate_;

  std::vector<double> reference_frame;
  reference_frame.assign(6,0);

  std::vector<double> reference_frame_minor;
  reference_frame_minor.assign(6,0);


  rw::math::Transform3D<> temp_inv_;

  double align_angle_z_ = 0;

  tf_base_to_tcp_ = Transform3D<> (Vector3D<>(robot_initial_pose_[0], robot_initial_pose_[1], robot_initial_pose_[2]), EAA<>(robot_initial_pose_[3], robot_initial_pose_[4], robot_initial_pose_[5]).toRotation3D());
  tf_base_to_start_ = Transform3D<> (Vector3D<>(temp_reference_frame_start[0], temp_reference_frame_start[1], temp_reference_frame_start[2]), EAA<>(robot_initial_pose_[3], robot_initial_pose_[4], robot_initial_pose_[5]).toRotation3D());
  tf_base_to_end_ = Transform3D<> (Vector3D<>(temp_reference_frame_end[0], temp_reference_frame_end[1], temp_reference_frame_end[2]), EAA<>(robot_initial_pose_[3], robot_initial_pose_[4], robot_initial_pose_[5]).toRotation3D());


  temp_inv_ = tf_base_to_tcp_;
  temp_inv_.invMult(temp_inv_, tf_base_to_start_);
  tf_tcp_to_start_ = temp_inv_;

  temp_inv_ = tf_base_to_tcp_;
  temp_inv_.invMult(temp_inv_, tf_base_to_end_);
  tf_tcp_to_end_ = temp_inv_;

  tf_tcp_to_direction_.P() = -(tf_tcp_to_end_.P() - tf_tcp_to_start_.P());

  align_angle_z_ = atan2(tf_tcp_to_direction_.P()[1],tf_tcp_to_direction_.P()[0]);


  if(align_angle_z_ > 69*DEGREE2RADIAN)
    align_angle_z_ = 69*DEGREE2RADIAN;
  if(align_angle_z_ < -69*DEGREE2RADIAN)
    align_angle_z_ = -69*DEGREE2RADIAN;

  tf_tcp_to_rotate_ =  Transform3D<> (Vector3D<>(0,0,0), RPY<>(align_angle_z_,0,0).toRotation3D());

  reference_frame = temp_reference_frame_start; // main

  tf_base_to_tcp_rotated_ = tf_base_to_tcp_*tf_tcp_to_rotate_;

  reference_frame_minor = temp_reference_frame_end;

  if(!command.compare("master"))
  {
    tf_base_to_start_.R() = tf_base_to_tcp_.R();
    robot_task_->initialize_reference_frame(tf_base_to_start_, tf_base_to_end_);
  }
  else
  {
    tf_base_to_start_.R() = tf_base_to_tcp_rotated_.R();
    tf_base_to_end_.R() = tf_base_to_tcp_.R();
    robot_task_->initialize_reference_frame(tf_base_to_start_, tf_base_to_end_);
  }
  desired_pose_vector_ = robot_task_ -> get_current_pose();
  std::cout << robot_name_ <<"::  NEW initialize !! " << std::endl;
}

bool TaskStrategy::tasks(std::string command) // first for only two pulleys
{
  if(previous_task_command_.compare(command))
  {
    sub_tasks_ = 0;
    robot_task_->clear_phase();

  }

  if(!command.compare(""))
    return false;
  else
  {
    if(!command.compare("master"))
    {
      robot_motion_->set_all_phases_(master_way_points_numbers_);
      master_robot();

      if(sub_tasks_ == 1)
        finish_task_ = 1;
    }

    if(!command.compare("slave"))
    {
      robot_motion_->set_all_phases_(slave_way_points_numbers_);
      slave_robot();
    }

    previous_phase_ = robot_task_->get_phases_();
    robot_motion_->generate_trajectory();
    robot_motion_->check_phases();
  }
  previous_task_command_ = command;

  return true;
}
void TaskStrategy::master_robot()
{
  all_phases_ = master_way_points_numbers_;
  static int motion_phases_ = 0;

  if(sub_tasks_ == 0 && robot_task_->get_phases_() == master_way_points_numbers_)
  {
    sub_tasks_++;
    return;
  }
  if(sub_tasks_ == 1) // pulley numbers
    return;

  if(previous_phase_ != robot_task_->get_phases_())
  {
    motion_phases_ = robot_task_->get_phases_();

  if(robot_motion_->get_phases_() == 3)
    {
      gripper_move_values = 12;
    }

    if(robot_motion_->get_phases_() == 1)
    {
      for(int num = 0; num < 3; num ++)
      {
        desired_force_torque_vector_[num] = 0;
      }
      gripper_move_values = 6;

      desired_belt_[0] = master_way_points_[1][0];
      desired_belt_[1] = master_way_points_[1][1];
      desired_belt_[2] = master_way_points_[1][2];

      robot_motion_->estimation_of_belt_position(desired_belt_); ////bearing frame
      robot_motion_->insert_into_groove(RPY<>(master_way_points_[1][3],master_way_points_[1][4],master_way_points_[1][5]));

      return;
    }
    robot_motion_->motion_to_desired_pose(contact_check_, master_way_points_[motion_phases_][0],master_way_points_[motion_phases_][1],master_way_points_[motion_phases_][2], RPY<>(master_way_points_[motion_phases_][3],master_way_points_[motion_phases_][4],master_way_points_[motion_phases_][5]),master_way_points_[motion_phases_][6]); //bearing frame
  }
}
void TaskStrategy::slave_robot() // for robot A
{
  static int motion_phases_ = 0;
  all_phases_ = slave_way_points_numbers_;

  if(sub_tasks_ == 0 && robot_task_->get_phases_() == slave_way_points_numbers_)
  {
    position_x_controller_->set_smooth_gain_time(2.5);
    position_y_controller_->set_smooth_gain_time(2.5);
    position_z_controller_->set_smooth_gain_time(2.5);
    force_x_compensator_->set_smooth_gain_time(2.5);
    force_y_compensator_->set_smooth_gain_time(2.5);
    force_z_compensator_->set_smooth_gain_time(2.5);

    cout << desired_force_torque_vector_ << "finished " << endl;

    sub_tasks_++;
    return;
  }
  if(sub_tasks_ == 1) // pulley numbers
  {
    return;
  }
  if(previous_phase_ != robot_task_->get_phases_())
  {

    motion_phases_ = robot_task_->get_phases_();

    robot_motion_->motion_to_desired_pose_big(contact_check_, slave_way_points_[motion_phases_][0],slave_way_points_[motion_phases_][1],slave_way_points_[motion_phases_][2], RPY<>(slave_way_points_[motion_phases_][3],slave_way_points_[motion_phases_][4],slave_way_points_[motion_phases_][5]),slave_way_points_[motion_phases_][6]); //bearin

    cout << "motion! " << endl;

    for(int num = 0; num < 3; num ++)
    {
      desired_force_torque_vector_[num] = force_vector_[motion_phases_][num];
    }
  }
}

void TaskStrategy::check_phases()
{
  pre_phases_ = phases_;
  if(robot_motion_->is_moving_check() == false && phases_ < all_phases_)
  {
    phases_ ++;
  }
}



double TaskStrategy::get_gripper_move_values()
{
  return  gripper_move_values;
}

void TaskMotion::estimation_of_belt_position(rw::math::Vector3D<> desired_position)
{
  temp_ = tf_base_to_bearing_start_;
  temp_.invMult(temp_, tf_current_pose_);
  tf_bearing_to_master_robot_ = temp_;

  //bearing robot point
  ref_belt_[0] = 0;
  ref_belt_[1] = tf_bearing_to_master_robot_.P()[1] + 0.008; // same in two robot
  ref_belt_[2] = tf_bearing_to_master_robot_.P()[2];

  desired_groove_position_ = desired_position;

  std::cout << " init_current_belt_  :"<< ref_belt_ << std::endl;

  tf_bearing_ref_belt_.P() = ref_belt_;
  tf_bearing_desired_groove_.P() = desired_groove_position_;

  std::cout << " tf_bearing_to_master_robot_x :"<< tf_bearing_to_master_robot_.P()[0] << std::endl;
  std::cout << " tf_bearing_to_master_robot_y :"<< tf_bearing_to_master_robot_.P()[1] << std::endl;
  std::cout << " tf_bearing_to_master_robot_z :"<< tf_bearing_to_master_robot_.P()[2] << std::endl;

  error_ = tf_bearing_desired_groove_.P() - tf_bearing_ref_belt_.P();

  modified_master_robot_ = tf_bearing_to_master_robot_.P() + error_;

  std::cout << " tf_desired_pose_x :"<< modified_master_robot_[0] << std::endl;
  std::cout << " tf_desired_pose_y :"<< modified_master_robot_[1] << std::endl;
  std::cout << " tf_desired_pose_z :"<< modified_master_robot_[2] << std::endl;
}
