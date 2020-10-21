/*
 * task_motion.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "task_motion.h"

TaskMotion::TaskMotion()
{

}
TaskMotion::~TaskMotion()
{

}

void TaskMotion::initialize(double control_time_)
{
  robot_traj_ = std::make_shared<EndEffectorTraj>();
  robot_traj_->set_control_time(control_time_);

  desired_pose_matrix_.resize(6,8);
  desired_pose_matrix_.fill(0);

  // initial pose load
  desired_pose_matrix_(0,7) = 5;
  desired_pose_matrix_(1,7) = 5;
  desired_pose_matrix_(2,7) = 5;
  desired_pose_matrix_(3,7) = 5;
  desired_pose_matrix_(4,7) = 5;
  desired_pose_matrix_(5,7) = 5;

  current_pose_vector_.resize(6);
  current_force_torque_vector_.resize(6);
}
void TaskMotion::motion_to_desired_pose(Transform3D<> reference_frame, double x, double y, double z, RPY<> tcp_rpy_, double time)
{
  // output always has to be points in relative to base frame (global)

  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());

  tf_desired_pose_ = reference_frame*tf_tcp_desired_pose_;

  desired_pose_matrix_(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
  desired_pose_matrix_(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
  desired_pose_matrix_(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

  desired_pose_matrix_(3,1) = RPY<> (tf_desired_pose_.R())[0]; //
  desired_pose_matrix_(4,1) = RPY<> (tf_desired_pose_.R())[1]; //
  desired_pose_matrix_(5,1) = RPY<> (tf_desired_pose_.R())[2]; //

  //std::cout << " desired_pose_matrix(3,1):"<< desired_pose_matrix(3,1)*RADIAN2DEGREE << std::endl;
  //std::cout << " desired_pose_matrix(4,1):"<< desired_pose_matrix(4,1)*RADIAN2DEGREE << std::endl;
  //std::cout << " desired_pose_matrix(5,1):"<< desired_pose_matrix(5,1)*RADIAN2DEGREE << std::endl;

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix_(num,7) = time;
  }
}
void TaskMotion::generate_trajectory()
{
  robot_traj_->cal_end_point_to_rad(desired_pose_matrix_);

  for(int num = 0; num <6 ; num ++)
  {
    current_pose_vector_[num] = robot_traj_->get_traj_results()(num,0);
  }
}
void TaskMotion::set_initial_pose(double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  tf_current_pose_ = Transform3D<> (Vector3D<>(x, y, z), EAA<>(axis_x, axis_y, axis_z).toRotation3D()); // EAA


  current_pose_vector_[0] = x;
  current_pose_vector_[1] = y;
  current_pose_vector_[2] = z;
  current_pose_vector_[3] = RPY<> (tf_current_pose_.R())[0];
  current_pose_vector_[4] = RPY<> (tf_current_pose_.R())[1];
  current_pose_vector_[5] = RPY<> (tf_current_pose_.R())[2];

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix_(num,0) =  current_pose_vector_[num];
    desired_pose_matrix_(num,1) =  current_pose_vector_[num];
    robot_traj_->current_pose_change(num,0) = current_pose_vector_[num];
  }

  robot_traj_->cal_end_point_tra_px->current_pose = current_pose_vector_[0];
  robot_traj_->cal_end_point_tra_py->current_pose = current_pose_vector_[1];
  robot_traj_->cal_end_point_tra_pz->current_pose = current_pose_vector_[2];
  robot_traj_->cal_end_point_tra_alpha->current_pose = current_pose_vector_[3];
  robot_traj_->cal_end_point_tra_betta->current_pose = current_pose_vector_[4];
  robot_traj_->cal_end_point_tra_kamma->current_pose = current_pose_vector_[5];
}
void TaskMotion::set_current_pose_eaa(double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  tf_current_pose_ = Transform3D<> (Vector3D<>(x, y, z), EAA<>(axis_x, axis_y, axis_z).toRotation3D()); // EAA
}
void TaskMotion::set_current_pose_rpy(double x, double y, double z, double yaw, double pitch, double roll)
{
  tf_current_pose_ = Transform3D<> (Vector3D<>(x, y, z), RPY<>(yaw, pitch, roll).toRotation3D()); // RPY
}
void TaskMotion::stop_motion()
{
  desired_pose_matrix_.fill(0);
  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix_(num,0) = current_pose_vector_[num];
    desired_pose_matrix_(num,1) = current_pose_vector_[num];
    desired_pose_matrix_(num,7) = 1;
  }
  robot_traj_->stop_trajectory();
}
bool TaskMotion::is_moving_check()
{
  return robot_traj_->is_moving_check;
}
std::vector<double> TaskMotion::get_current_pose()
{
  return current_pose_vector_;
}


