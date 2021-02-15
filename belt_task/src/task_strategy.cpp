/*
 * task_strategy.cpp
 *
 *  Created on: Oct 20, 2020
 *      Author: yik
 */
#include "task_strategy.h"

TaskStrategy::TaskStrategy()
{
}
TaskStrategy::TaskStrategy(std::string robot_name)
{
  initial_pose_vector_.assign(6,0);
  current_way_points_.assign(7,0);
  current_desired_force_vector_.assign(3,0);
  first_part_.assign(6,0);
  second_part_.assign(6,0);
  pre_phases_ = -1;
  phases_ = -1;
  all_phases_ = 0;

  master_way_points_numbers_ = 0;
  slave_way_points_numbers_ = 0;
  desired_force_values_numbers_ = 0;

  finish_tasks_ = 0;
  sub_tasks_ = 0;

  type_ = "";
  pre_type_ = "";

  //grippers
  gripper_move_values_= 0;
  robot_name_ = robot_name;
  smooth_gain_change_time_ = 2.5;

  //is_moving_check_
  is_moving_check_ = false;
}
TaskStrategy::~TaskStrategy()
{

}
void TaskStrategy::initialize(const std::string &path)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); //
    file_path_ = path;

  }catch(const std::exception& e) //
  {
    std::cout << "Fail to load yaml file!" << std::endl;
    return;
  }
  YAML::Node part_position_node = doc["part_position"];
  YAML::Node robot_initial_pose = doc["robot_initial_pose"];
  YAML::Node desired_force_value_node = doc["desired_force_value"];

  std::vector<double> temp_points_;
  int temp_points_numbers_ = 0;

  temp_points_.clear();
  for (YAML::iterator it = desired_force_value_node.begin(); it != desired_force_value_node.end(); ++it)
  {
    desired_force_values_numbers_ ++;
    temp_points_numbers_ = it->first.as<int>();

    for(int num = 0; num < 3; num++)
    {
      temp_points_.push_back(it->second[num].as<double>());
    }

    desired_force_values_[temp_points_numbers_] = temp_points_;


    temp_points_.clear();
  }

  initial_pose_vector_[0] = robot_initial_pose[0].as<double>();
  initial_pose_vector_[1] = robot_initial_pose[1].as<double>();
  initial_pose_vector_[2] = robot_initial_pose[2].as<double>();
  initial_pose_vector_[3] = robot_initial_pose[3].as<double>();
  initial_pose_vector_[4] = robot_initial_pose[4].as<double>();
  initial_pose_vector_[5] = robot_initial_pose[5].as<double>();
}
void TaskStrategy::assign_parts(std::string master, std::string slave)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(file_path_.c_str()); //

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

    std::cout << robot_name_ << " " << temp_points_numbers_ <<"::"<< master_way_points_[temp_points_numbers_]  << std::endl;
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

    std::cout << robot_name_ << " " <<  temp_points_numbers_ <<"::"<< slave_way_points_[temp_points_numbers_]  << std::endl;
  }
}
void TaskStrategy::initialize_reference_frame()
{
  rw::math::Transform3D<> temp_inv_;
  double align_angle_z_ = 0;
  tf_base_to_tcp_ = Transform3D<> (Vector3D<>(initial_pose_vector_[0], initial_pose_vector_[1], initial_pose_vector_[2]), EAA<>(initial_pose_vector_[3], initial_pose_vector_[4], initial_pose_vector_[5]).toRotation3D());

  //  temp_inv_ = tf_base_to_tcp_;
  //  temp_inv_.invMult(temp_inv_, tf_base_to_start_);
  //  tf_tcp_to_start_ = temp_inv_;
  //
  //  temp_inv_ = tf_base_to_tcp_;
  //  temp_inv_.invMult(temp_inv_, tf_base_to_end_);
  //  tf_tcp_to_end_ = temp_inv_;

  //  tf_tcp_to_direction_.P() = -(tf_tcp_to_end_.P() - tf_tcp_to_start_.P());
  //
  //  align_angle_z_ = atan2(tf_tcp_to_direction_.P()[1],tf_tcp_to_direction_.P()[0]);
  //
  //  if(align_angle_z_ > 69*DEGREE2RADIAN)
  //    align_angle_z_ = 69*DEGREE2RADIAN;
  //  if(align_angle_z_ < -69*DEGREE2RADIAN)
  //    align_angle_z_ = -69*DEGREE2RADIAN;
  //
  //  tf_tcp_to_rotate_ =  Transform3D<> (Vector3D<>(0,0,0), RPY<>(align_angle_z_,0,0).toRotation3D());
  //  tf_base_to_tcp_rotated_ = tf_base_to_tcp_*tf_tcp_to_rotate_;
  //
  //
  //  if(!type_.compare("master"))
  //  {
  //    tf_base_to_start_.R() = tf_base_to_tcp_.R();
  //  }
  //  else
  //  {
  //    tf_base_to_start_.R() = tf_base_to_tcp_rotated_.R();
  //    tf_base_to_end_.R() = tf_base_to_tcp_.R();
  //  }

  //  current_reference_frame_ = tf_base_to_tcp_;
  std::cout << robot_name_ <<"::  NEW initialize !! " << tf_base_to_tcp_ << std::endl;
}
void TaskStrategy::estimation_of_belt_position()
{
  Transform3D<> temp_;
  Transform3D<> tf_bearing_to_robot_;
  Transform3D<> tf_bearing_to_cur_belt_;
  Transform3D<> tf_bearing_to_desired_point_;
  rw::math::Vector3D<> error_;
  rw::math::Vector3D<> cur_belt_;

  temp_ = tf_base_to_start_;
  temp_.invMult(temp_, tf_current_pose_);
  tf_bearing_to_robot_ = temp_;

  //bearing robot point
  cur_belt_[0] = 0;
  cur_belt_[1] = tf_bearing_to_robot_.P()[1] + 0.008; // same in two robot
  cur_belt_[2] = tf_bearing_to_robot_.P()[2];

  std::cout << " init_current_belt_  :"<< cur_belt_ << std::endl;

  tf_bearing_to_cur_belt_.P() = cur_belt_;
  tf_bearing_to_desired_point_.P() = desired_groove_position_;

  std::cout << " tf_bearing_to_master_robot_x :"<< tf_bearing_to_robot_.P()[0] << std::endl;
  std::cout << " tf_bearing_to_master_robot_y :"<< tf_bearing_to_robot_.P()[1] << std::endl;
  std::cout << " tf_bearing_to_master_robot_z :"<< tf_bearing_to_robot_.P()[2] << std::endl;

  error_ = tf_bearing_to_desired_point_.P() - tf_bearing_to_cur_belt_.P();

  insert_belt_way_points_ = tf_bearing_to_robot_.P() + error_;

  std::cout << " tf_desired_pose_x :"<< insert_belt_way_points_[0] << std::endl;
  std::cout << " tf_desired_pose_y :"<< insert_belt_way_points_[1] << std::endl;
  std::cout << " tf_desired_pose_z :"<< insert_belt_way_points_[2] << std::endl;
}
void TaskStrategy::decision_of_function()
{
  if(!type_.compare(""))
  {
    if(first_part_[1] > second_part_[1])
    {
      type_ = "slave"; // small
      assign_parts("master_pulley_small", "slave_pulley_small");
      tf_base_to_start_.P()[0] = second_part_[0];
      tf_base_to_start_.P()[1] = second_part_[1];
      tf_base_to_start_.P()[2] = second_part_[2];

      tf_base_to_end_.P()[0] = first_part_[0];
      tf_base_to_end_.P()[1] = first_part_[1];
      tf_base_to_end_.P()[2] = first_part_[2];
    }
    if(first_part_[1] < second_part_[1])
    {
      type_ = "master";// big
      assign_parts("master_pulley_big", "slave_pulley_big");
      tf_base_to_start_.P()[0] = first_part_[0];
      tf_base_to_start_.P()[1] = first_part_[1];
      tf_base_to_start_.P()[2] = first_part_[2];

      tf_base_to_end_.P()[0] = second_part_[0];
      tf_base_to_end_.P()[1] = second_part_[1];
      tf_base_to_end_.P()[2] = second_part_[2];
    }
  }
  else
  {
    if(!type_.compare("master"))
    {
      assign_parts("master_pulley_big", "slave_pulley_big");
      tf_base_to_start_.P()[0] = first_part_[0];
      tf_base_to_start_.P()[1] = first_part_[1];
      tf_base_to_start_.P()[2] = first_part_[2];

      tf_base_to_end_.P()[0] = second_part_[0];
      tf_base_to_end_.P()[1] = second_part_[1];
      tf_base_to_end_.P()[2] = second_part_[2];
    }

    if(!type_.compare("slave"))
    {
      assign_parts("master_pulley_small", "slave_pulley_small");
      tf_base_to_start_.P()[0] = second_part_[0];
      tf_base_to_start_.P()[1] = second_part_[1];
      tf_base_to_start_.P()[2] = second_part_[2];

      tf_base_to_end_.P()[0] = first_part_[0];
      tf_base_to_end_.P()[1] = first_part_[1];
      tf_base_to_end_.P()[2] = first_part_[2];
    }
  }
}
bool TaskStrategy::tasks() // first for only two pulleys
{
  check_phases();
  if(pre_type_.compare(type_))
  {
    pre_phases_ = -1;
    phases_ = -1;
    all_phases_ = 0;
  }

  if(!type_.compare(""))
    return false;
  else
  {
    if(!type_.compare("master"))
    {
      master_robot();

      if(sub_tasks_ == 1)
        finish_tasks_ = 1;
    }

    if(!type_.compare("slave"))
    {
      slave_robot();
    }
  }
  pre_type_ = type_;
  return true;
}
void TaskStrategy::master_robot()
{
  static int motion_phases_ = 0;
  all_phases_ = master_way_points_numbers_;

  if(sub_tasks_ == 0 && phases_ == all_phases_)
  {
    sub_tasks_++;
    return;
  }
  if(sub_tasks_ == 1) // pulley numbers
    return;

  if(pre_phases_ != phases_)
  {
    motion_phases_ = phases_;
    std::cout << "Master motion! " << std::endl;
    current_reference_frame_ = tf_base_to_start_;

    if(phases_ == 3)
    {
      gripper_move_values_ = 12;
    }

    if(phases_ == 1)
    {
      for(int num = 0; num < 3; num ++)
      {
        current_desired_force_vector_[num] = 0;
      }
      gripper_move_values_ = 6;

      desired_groove_position_[0] = master_way_points_[1][0];
      desired_groove_position_[1] = master_way_points_[1][1];
      desired_groove_position_[2] = master_way_points_[1][2];

      estimation_of_belt_position(); ////bearing frame

      master_way_points_[1][0] = insert_belt_way_points_[0];
      master_way_points_[1][1] = insert_belt_way_points_[1];
      master_way_points_[1][2] = insert_belt_way_points_[2];
    }

    for(int num = 0; num < 7; num ++)
    {
      current_way_points_[num] = master_way_points_[phases_][num];
    }
    std::cout << " current_way_points_ :"<< current_way_points_ << std::endl;
  }
}
void TaskStrategy::slave_robot()
{
  static int motion_phases_ = 0;
  all_phases_ = slave_way_points_numbers_;

  if(sub_tasks_ == 0 && phases_ == slave_way_points_numbers_)
  {
    smooth_gain_change_time_ = 2.5;
    return;
  }
  if(pre_phases_ != phases_)
  {

    motion_phases_ = phases_;
    std::cout << "slave motion! " << std::endl;
    current_reference_frame_ = tf_base_to_end_;

    for(int num = 0; num < 3; num ++)
    {
      //force vector
      current_desired_force_vector_[num] = desired_force_values_[motion_phases_][num];
    }

    for(int num = 0; num < 7; num ++)
    {
      current_way_points_[num] = slave_way_points_[phases_][num];
    }
  }
}
void TaskStrategy::check_phases()
{
  pre_phases_ = phases_;
  if(is_moving_check_ == false && phases_ < all_phases_)
  {
    phases_ ++;
  }
}
void TaskStrategy::set_is_moving_check(bool check)
{
  is_moving_check_ = check;
}
void TaskStrategy::set_robot_current_tf(Transform3D<> tf_current_pose)
{
  tf_current_pose_ = tf_current_pose;
}
void TaskStrategy::set_parts_data(std::vector<double> first_part, std::vector<double> second_part)
{
  first_part_ = first_part;
  second_part_ = second_part;

  tf_base_to_start_ = Transform3D<> (Vector3D<>(first_part[0],first_part[1],first_part[2]), EAA<>(first_part[3],first_part[4],first_part[5]).toRotation3D());
  tf_base_to_end_ = Transform3D<> (Vector3D<>(second_part_[0],second_part_[1],second_part_[2]), EAA<>(second_part_[3],second_part_[4],second_part_[5]).toRotation3D());


  //  Transform3D<> tf_start_tcp;
  //  Transform3D<> tf_base_to_start_tcp;
  //
  //  tf_start_tcp = Transform3D<> (Vector3D<>(0.03,0,-0.02), RPY<>(0,-25*DEGREE2RADIAN,0).toRotation3D()); // b
  //  tf_base_to_start_tcp = tf_base_to_start_*tf_start_tcp;
  //
  //  std::cout << "A:0  " << (tf_base_to_start_tcp.P())[0] << std::endl;
  //  std::cout << "A:1  " << (tf_base_to_start_tcp.P())[1] << std::endl;
  //  std::cout << "A:2  " << (tf_base_to_start_tcp.P())[2] << std::endl;
  //
  //  std::cout << "A:3  " << EAA<> (tf_base_to_start_tcp.R())[0] << std::endl;
  //  std::cout << "A:4  " << EAA<> (tf_base_to_start_tcp.R())[1] << std::endl;
  //  std::cout << "A:5  " << EAA<> (tf_base_to_start_tcp.R())[2] << std::endl;


  //  tf_start_tcp = Transform3D<> (Vector3D<>(0.03,0,-0.02), RPY<>(0,-25*DEGREE2RADIAN,0).toRotation3D()); // b
  //  tf_base_to_start_tcp = tf_base_to_start_*tf_base_to_start_tcp;
  //
  //  std::cout << "B:0  " << (tf_base_to_start_tcp.P())[0] << std::endl;
  //  std::cout << "B:1  " << (tf_base_to_start_tcp.P())[1] << std::endl;
  //  std::cout << "B:2  " << (tf_base_to_start_tcp.P())[2] << std::endl;
  //
  //  std::cout << "B:3  " << EAA<> (tf_base_to_start_tcp.R())[0] << std::endl;
  //  std::cout << "B:4  " << EAA<> (tf_base_to_start_tcp.R())[1] << std::endl;
  //  std::cout << "B:5  " << EAA<> (tf_base_to_start_tcp.R())[2] << std::endl;


  //iros
  //decision_of_function();
  initialize_reference_frame();
  //assign_parts("master_pulley_big", "slave_pulley_big");
}
void TaskStrategy::input_paths(geometry_msgs::PoseArray paths_)
{

  Transform3D<> temp_pose_;
  temp_pose_ = Transform3D<> (Vector3D<>(0, 0, 0), Quaternion<>(0,0,0,0).toRotation3D());// xyz w

  std::vector<double> temp_points_;
  int temp_points_numbers_ = 0;

  master_way_points_numbers_ = 0;

  double roll_x, pitch_y, yaw_z;
  roll_x = 0;
  pitch_y = 0;
  yaw_z = 0;

  if(paths_.poses.size() == 0)
    return;

  for(int num = 0; num < paths_.poses.size(); num ++)
  {
    master_way_points_numbers_ ++;

    temp_points_.push_back(paths_.poses[num].position.x);
    temp_points_.push_back(paths_.poses[num].position.y);
    temp_points_.push_back(paths_.poses[num].position.z);

    temp_pose_ = Transform3D<> (Vector3D<>(0, 0, 0), Quaternion<>(paths_.poses[num].orientation.x,paths_.poses[num].orientation.y,paths_.poses[num].orientation.z,paths_.poses[num].orientation.w).toRotation3D());// xyz w


    yaw_z = RPY<>(temp_pose_.R())[0]; // z
    pitch_y = RPY<>(temp_pose_.R())[1]; // y
    roll_x = RPY<>(temp_pose_.R())[2]; // x

    temp_points_.push_back(yaw_z);
    temp_points_.push_back(pitch_y);
    temp_points_.push_back(roll_x);

    temp_points_.push_back(0.5); // should be modified

    master_way_points_[num] = temp_points_;

    temp_points_.clear();

    std::cout << robot_name_ << " " << num  <<"::"<< master_way_points_[num]  << std::endl;
  }
}
bool TaskStrategy::get_finish_task_check()
{
  return finish_tasks_;
}
double TaskStrategy::get_controller_smooth_gain_change()
{
  return smooth_gain_change_time_;
}
double TaskStrategy::get_gripper_move_values()
{
  return  gripper_move_values_;
}
std::string TaskStrategy::get_type()
{
  return type_;
}
void TaskStrategy::set_type(std::string type)
{
  type_ = type;
}
Transform3D<> TaskStrategy::get_current_reference_frame_()
        {
  return current_reference_frame_;
        }
std::vector<double> TaskStrategy::get_current_way_points_()
{
  return current_way_points_;
}
std::vector<double> TaskStrategy::get_current_desired_force_vector_()
{
  return current_desired_force_vector_;
}
