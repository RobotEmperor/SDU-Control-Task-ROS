/*
 * object_estimation.cpp
 *
 *  Created on: Jan 21, 2021
 *      Author: yik
 */

#include "object_estimation.h"

ObejectEstimation::ObejectEstimation()
{
  spring_constant_k_ = 0;
  damper_constant_c_ = 0;

  objects_force_magnitude_ = 0;
  initial_grab_point_magnitude_ = 0;
  current_grab_point_magnitude_ = 0;
}
ObejectEstimation::~ObejectEstimation()
{
}
void ObejectEstimation::initialize(int all_point_number, const std::string &path)
{
  std::vector<double> temp_vector;
  temp_vector.assign(6, 0);

  for(int num_ = 0; num_ < all_point_number; num_ ++)
  {
    current_object_pose_vector_.push_back(temp_vector);
    current_object_force_torque_vector_.push_back(temp_vector);
  }

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
  spring_constant_k_= doc["spring_constant_k"].as<double>();
  damper_constant_c_ = doc["damper_constant_c"].as<double>();
}
void ObejectEstimation::model_estimation()
{
  //set frame to parts frame
  rw::math::Transform3D<> tf_a_parts;

  rw::math::Transform3D<> tf_big_to_small_;
  rw::math::Transform3D<> tf_a_to_small_;

  rw::math::Transform3D<> tf_small_to_init_ee;
  rw::math::Transform3D<> tf_small_to_final_ee;

  tf_a_parts =  Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D());

  tf_big_to_small_ = Transform3D<> (Vector3D<>(-0.130, 0.007, 0), RPY<> (0,0,0).toRotation3D()); // RPY
  tf_a_to_small_ = tf_a_parts * tf_big_to_small_;

  tf_small_to_init_ee = inverse(tf_a_to_small_)*initial_grabs_tf_pose_;
  tf_small_to_final_ee = inverse(tf_a_to_small_)*current_grabs_tf_pose_;

  // constraint function
  static std::vector<double> force_vector_;

  force_vector_.push_back(spring_constant_k_*(tf_small_to_init_ee.P()[0] -  tf_small_to_final_ee.P()[0]));
  force_vector_.push_back(spring_constant_k_*(tf_small_to_init_ee.P()[1] -  tf_small_to_final_ee.P()[1]));
  force_vector_.push_back(spring_constant_k_*(tf_small_to_init_ee.P()[2] -  tf_small_to_final_ee.P()[2]));

//  static std::vector<double> force_vector_;
//
//  force_vector_.push_back(spring_constant_k_*(initial_grabs_pose_[0][0] - current_grabs_pose_[0][0]));
//  force_vector_.push_back(spring_constant_k_*(initial_grabs_pose_[0][1] - current_grabs_pose_[0][1]));
//  force_vector_.push_back(spring_constant_k_*(initial_grabs_pose_[0][2] - current_grabs_pose_[0][2]));

  current_grab_point_magnitude_ = sqrt(pow(force_vector_[0],2) + pow(force_vector_[1],2) + pow(force_vector_[2],2));

  for(long unsigned int num_inner_ = 0; num_inner_ < 3; num_inner_ ++)
  {
    current_object_force_torque_vector_[0][num_inner_]=force_vector_[num_inner_];
  }
  //check
  //std::cout << force_unit_vector_ << std::endl;
//  std::cout << "----------------------------------------------------------"<< std::endl;
//  std::cout << "force X ::  " << current_grab_point_magnitude_ << std::endl;
//  std::cout << "force X ::  " << current_object_force_torque_vector_[0][0] << std::endl;
//  std::cout << "force Y ::  " << current_object_force_torque_vector_[0][1] << std::endl;
//  std::cout << "force Z ::  " << current_object_force_torque_vector_[0][2] << std::endl;
  force_vector_.clear();
}
void ObejectEstimation::set_current_grab_poses(std::vector<std::vector<double>> current_grabs_poses)
{
  current_grabs_pose_ = current_grabs_poses;
}
void ObejectEstimation::set_initial_grab_poses(std::vector<std::vector<double>> initial_grabs_poses) // from 0 to 1
{
  initial_grabs_pose_ = initial_grabs_poses;
  //std::cout << "initial_grabs_pose_ ::  " << initial_grabs_pose_[0] << std::endl;

  //  for(long unsigned int num_ = 0; num_ < initial_grabs_poses.size() - 1 ;num_ ++)
  //  {
  //    initial_grab_point_magnitude_ = sqrt(pow(2,initial_grabs_poses[num_][0] - initial_grabs_poses[num_+1][0])
  //        + pow(2,initial_grabs_poses[num_][1] - initial_grabs_poses[num_+1][1])
  //        + pow(2,initial_grabs_poses[num_][2] - initial_grabs_poses[num_+1][2]));
  //  }
}
void ObejectEstimation::set_current_grab_poses(rw::math::Transform3D<> current_grabs_poses)
{
  current_grabs_tf_pose_ = current_grabs_poses;
}
void ObejectEstimation::set_initial_grab_poses(rw::math::Transform3D<> initial_grabs_poses) // from 0 to 1
{
  initial_grabs_tf_pose_ = initial_grabs_poses;
}
void ObejectEstimation::set_current_grab_vels(std::vector<std::vector<double>> current_grabs_vel)
{
  current_grabs_vel_ = current_grabs_vel;
}
void ObejectEstimation::set_initial_grab_vels(std::vector<std::vector<double>> initial_grabs_vel)
{
  initial_grabs_vel_ = initial_grabs_vel;
}
std::vector<double> ObejectEstimation::get_current_object_pose(int point_number)
{
  return current_object_pose_vector_[point_number];
}
std::vector<double> ObejectEstimation::get_current_object_force(int point_number)
{
  return current_object_force_torque_vector_[point_number];
}
double ObejectEstimation::get_current_object_force()
{
  return current_grab_point_magnitude_;
}


