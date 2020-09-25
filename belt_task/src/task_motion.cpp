/*
 * task_motion.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "task_motion.h"


TaskMotion::TaskMotion()
{
  path_angle_ = 0;
  path_y_= 0;
  change_path_x_ = 0.001;
  change_path_y_ = 0.001;
  change_path_z_ = 0.001;
  phases_= 0;
  pre_phases_ = 0;
  all_phases_ = 0;

  bigger_pulley_bearing_position.assign(6,0);
}
TaskMotion::~TaskMotion()
{

}

void TaskMotion::initialize(double control_time_, std::string load_path_)
{
  robot_traj = std::make_shared<EndEffectorTraj>();
  robot_traj->set_control_time(control_time_);

  current_point = -1; // wanna count from 0
  all_point = -1;
  tcp_all_point = -1;
  check_change = false;
  task_done = false;
  base_frame_ = true;

  desired_pose_matrix.resize(6,8);
  desired_pose_matrix.fill(0);

  // initial pose load
  desired_pose_matrix(0,7) = 5;
  desired_pose_matrix(1,7) = 5;
  desired_pose_matrix(2,7) = 5;
  desired_pose_matrix(3,7) = 5;
  desired_pose_matrix(4,7) = 5;
  desired_pose_matrix(5,7) = 5;

  current_pose_vector.resize(6);
  current_force_torque_vector.resize(6);

  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(load_path_.c_str());
  }catch(const std::exception& e)
  {
    std::cout << COLOR_RED_BOLD << "Fail to load data, yaml file!" << COLOR_RESET << std::endl;
    return;
  }

  // motion data load initialize//
  YAML::Node bigger_pulley_bearing_position_node = doc["pulley_bearing_position"];
  YAML::Node task_initial_position_node = doc["task_initial_position"];
  YAML::Node pulley_radious_node = doc["pulley_radious"];

  for(int num = 0; num < 6; num ++)
  {
    bigger_pulley_bearing_position[num] = bigger_pulley_bearing_position_node[num].as<double>();
  }

  for(int num = 0; num < 3; num ++)
    task_initial_position.push_back(task_initial_position_node[num].as<double>());
  for(int num = 3; num < 6; num ++)
    task_initial_position.push_back(task_initial_position_node[num].as<double>()*DEGREE2RADIAN);


  tf_base_to_bearing_ = Transform3D<> (Vector3D<>(bigger_pulley_bearing_position[0], bigger_pulley_bearing_position[1], bigger_pulley_bearing_position[2]), EAA<>(bigger_pulley_bearing_position[3], bigger_pulley_bearing_position[4], bigger_pulley_bearing_position[5]).toRotation3D());
  tf_bearing_to_init_ = Transform3D<> (Vector3D<>(task_initial_position[0], task_initial_position[1], task_initial_position[2]), RPY<>(task_initial_position[3],task_initial_position[4], task_initial_position[5]).toRotation3D());

  tf_base_to_init_task_ = tf_base_to_bearing_*tf_bearing_to_init_;

  for(int num = 0; num < 3; num ++)
    initial_robot_ee_position.push_back(tf_base_to_init_task_.P()[num]);
  for(int num = 0; num < 3; num ++)
    initial_robot_ee_position.push_back(EAA<>(tf_base_to_init_task_.R())[num]);

  set_initial_pose(initial_robot_ee_position[0], initial_robot_ee_position[1], initial_robot_ee_position[2], initial_robot_ee_position[3], initial_robot_ee_position[4], initial_robot_ee_position[5]); // set to be robot initial values

  for(int num = 0; num < 2; num ++)
  {
    radious_.push_back(pulley_radious_node[num].as<double>());
  }
  std::cout << "tf_base_to_init_task : " << tf_base_to_init_task_ << std::endl;
  std::cout << "initial_robot_ee_position : "<<initial_robot_ee_position << std::endl;
}
void TaskMotion::initialize_reference_frame(std::vector<double> reference_frame) // joint space
{
  initial_robot_ee_position.clear();
  bigger_pulley_bearing_position = reference_frame;

  tf_base_to_bearing_ = Transform3D<> (Vector3D<>(bigger_pulley_bearing_position[0], bigger_pulley_bearing_position[1], bigger_pulley_bearing_position[2]), EAA<>(bigger_pulley_bearing_position[3], bigger_pulley_bearing_position[4], bigger_pulley_bearing_position[5]).toRotation3D());
  tf_bearing_to_init_ = Transform3D<> (Vector3D<>(task_initial_position[0], task_initial_position[1], task_initial_position[2]), RPY<>(task_initial_position[3],task_initial_position[4], task_initial_position[5]).toRotation3D());

  tf_base_to_init_task_ = tf_base_to_bearing_*tf_bearing_to_init_;

  for(int num = 0; num < 3; num ++)
    initial_robot_ee_position.push_back(tf_base_to_init_task_.P()[num]);
  for(int num = 0; num < 3; num ++)
    initial_robot_ee_position.push_back(EAA<>(tf_base_to_init_task_.R())[num]);

  set_initial_pose(initial_robot_ee_position[0], initial_robot_ee_position[1], initial_robot_ee_position[2], initial_robot_ee_position[3], initial_robot_ee_position[4], initial_robot_ee_position[5]); // set to be robot initial values

  std::cout << "NEW tf_base_to_init_task : " << tf_base_to_init_task_ << std::endl;
  std::cout << "NEW initial_robot_ee_position : "<<initial_robot_ee_position << std::endl;
}
void TaskMotion::load_task_motion(std::string path_, std::string motion_)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path_.c_str());
  }catch(const std::exception& e)
  {
    std::cout << COLOR_RED_BOLD << "Fail to load data, yaml file!" << COLOR_RESET << std::endl;
    return;
  }

  // motion data load initialize//
  YAML::Node motion_start_time_node = doc["motion_start_time"];
  //YAML::Node motion_task_node = doc["motion_task"];

  std::vector<double> temp_motion_start_time_vector;
  std::vector<double> temp_motion_task_pose_vector;
  std::vector<double> temp_motion_task_vel_vector;

  int point_numbers;
  point_numbers = 0;

  for (YAML::iterator it = motion_start_time_node.begin(); it != motion_start_time_node.end(); ++it)
  {
    point_numbers = it->first.as<int>();
    temp_motion_start_time_vector.push_back(it->second[0].as<double>());
    motion_start_time_vector[point_numbers] = temp_motion_start_time_vector;
    temp_motion_start_time_vector.clear();

    for(int num = 0; num < 3; num++)
    {
      temp_motion_task_pose_vector.push_back(initial_robot_ee_position[num]);
      temp_motion_task_vel_vector.push_back(0);
    }
    for(int num = 3; num < 6; num++)
    {

      temp_motion_task_pose_vector.push_back(initial_robot_ee_position[num]);
      temp_motion_task_vel_vector.push_back(0);

    }
    motion_task_pose_vector[point_numbers] = temp_motion_task_pose_vector;
    all_point ++;
  }

  for(int num = 0; num < all_point+1 ; num++)
  {
    motion_task_init_vel_vector[num] = temp_motion_task_vel_vector;
    motion_task_final_vel_vector[num] = temp_motion_task_vel_vector;
  }

  temp_motion_start_time_vector.clear();
  temp_motion_task_pose_vector.clear();
  temp_motion_task_vel_vector.clear();

  init_all_point = all_point;
  init_motion_start_time_vector = motion_start_time_vector;
  init_motion_task_pose_vector = motion_task_pose_vector;
  init_motion_task_init_vel_vector = motion_task_init_vel_vector;
  init_motion_task_final_vel_vector = motion_task_final_vel_vector;
  all_point = -1;

  base_frame_ = true;
  std::cout << "LOAD initial pose Complete" << std::endl;
}

void TaskMotion::estimation_of_belt_position(rw::math::Vector3D<> desired_position)
{
  temp_ = tf_base_to_bearing_;
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
  //bearing robot point

  //    temp_ = tf_bearing_to_master_robot_;
  //    temp_.invMult(temp_, tf_bearing_ref_belt_);
  //    tf_master_robot_to_ref_belt_.P() = temp_.P();
  //
  //    temp_ = tf_bearing_to_master_robot_;
  //    temp_.invMult(temp_, tf_bearing_desired_groove_);
  //    tf_master_robot_to_desired_groove_.P()  = temp_.P();

  std::cout << " tf_bearing_to_master_robot_x :"<< tf_bearing_to_master_robot_.P()[0] << std::endl;
  std::cout << " tf_bearing_to_master_robot_y :"<< tf_bearing_to_master_robot_.P()[1] << std::endl;
  std::cout << " tf_bearing_to_master_robot_z :"<< tf_bearing_to_master_robot_.P()[2] << std::endl;


  error_ = tf_bearing_desired_groove_.P() - tf_bearing_ref_belt_.P();

  modified_master_robot_ = tf_bearing_to_master_robot_.P() + error_;

  std::cout << " tf_desired_pose_x :"<< modified_master_robot_[0] << std::endl;
  std::cout << " tf_desired_pose_y :"<< modified_master_robot_[1] << std::endl;
  std::cout << " tf_desired_pose_z :"<< modified_master_robot_[2] << std::endl;
}
void TaskMotion::insert_into_groove(RPY<> tcp_rpy_)
{
  insert_belt_into_pulley(0, modified_master_robot_[0], modified_master_robot_[1], modified_master_robot_[2], tcp_rpy_);
}
void TaskMotion::make_belt_robust(double radious)
{
  static RPY<> tcp_rpy_;
  static RPY<> pulley_frame_rpy_;
  static Vector3D<> pulley_frame_xyz_;

  tcp_rpy_ = RPY<>(0,0,0);

  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(radious, 0, 0), tcp_rpy_.toRotation3D());

  tf_desired_pose_ = tf_base_to_init_task_*tf_tcp_desired_pose_;

  desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
  desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
  desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

  desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
  desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
  desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

  for(int num = 0; num <6; num ++)
  {
    desired_pose_matrix(num,7) = 5;
  }
}
void TaskMotion::close_to_pulleys(double x,double y,double depth,RPY<> tcp_rpy_)
{

  // output always has to be points in relative to base frame (global)

  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, depth), tcp_rpy_.toRotation3D());

  tf_desired_pose_ = tf_base_to_bearing_*tf_tcp_desired_pose_;

  desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
  desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
  desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

  desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
  desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
  desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,7) = 2.5;
  }
}
void TaskMotion::insert_belt_into_pulley(bool contact_, double change_x, double change_y, double change_z, RPY<> tcp_rpy_)
{
  // output always has to be points in relative to base frame (global)


  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(change_x, change_y, change_z), tcp_rpy_.toRotation3D());

  tf_desired_pose_ = tf_base_to_bearing_*tf_tcp_desired_pose_;

  desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
  desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
  desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

  desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
  desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
  desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,7) = 7;
  }
}
void TaskMotion::rotate(double theta_)
{
  //  static RPY<> tcp_rpy_;
  //
  //  if(phases_ == 3)
  //  {
  //    tcp_rpy_ = RPY<>(axis_x,axis_y,axis_z);
  //
  //    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());
  //
  //    tf_desired_pose_ = tf_base_to_bearing_*tf_tcp_desired_pose_;
  //
  //  }
}
void TaskMotion::up_motion(bool contact_, double x, double y, double z,RPY<> tcp_rpy_)
{

  // output always has to be points in relative to base frame (global)

  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());

  tf_desired_pose_ = tf_base_to_bearing_*tf_tcp_desired_pose_;

  desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
  desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
  desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

  desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
  desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
  desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,7) = 2.5;
  }
}
void TaskMotion::finish_1(bool contact_, double x, double y, double z,RPY<> tcp_rpy_)
{

  // output always has to be points in relative to base frame (global)

  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());

  tf_desired_pose_ = tf_base_to_bearing_*tf_tcp_desired_pose_;

  desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
  desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
  desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

  desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
  desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
  desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,7) = 2.5;
  }
}
void TaskMotion::finish_2(bool contact_, double x, double y, double z,RPY<> tcp_rpy_)
{
  // output always has to be points in relative to base frame (global)

  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());

  tf_desired_pose_ = tf_base_to_bearing_*tf_tcp_desired_pose_;

  desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
  desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
  desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

  desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
  desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
  desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,7) = 2.5;
  }
}
void TaskMotion::motion_to_desired_pose(bool contact_, double x, double y, double z,RPY<> tcp_rpy_, double time)
{
  // output always has to be points in relative to base frame (global)

  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());

  tf_desired_pose_ = tf_base_to_bearing_*tf_tcp_desired_pose_;

  desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
  desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
  desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

  desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
  desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
  desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,7) = time;
  }
}
void TaskMotion::check_phases()
{
  pre_phases_ = phases_;
  if(robot_traj->is_moving_check == false && phases_ < all_phases_)
  {
    phases_ ++;
  }
}
void TaskMotion::generate_trajectory()
{
  robot_traj->cal_end_point_to_rad(desired_pose_matrix);

  for(int num = 0; num <6 ; num ++)
  {
    current_pose_vector[num] = robot_traj->get_traj_results()(num,0);
  }
}
void TaskMotion::set_initial_pose(double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  current_pose_vector[0] = x;
  current_pose_vector[1] = y;
  current_pose_vector[2] = z;
  current_pose_vector[3] = axis_x;
  current_pose_vector[4] = axis_y;
  current_pose_vector[5] = axis_z;

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,0) =  current_pose_vector[num];
    desired_pose_matrix(num,1) =  current_pose_vector[num];
    robot_traj->current_pose_change(num,0) = current_pose_vector[num];
  }

  robot_traj->cal_end_point_tra_px->current_pose = current_pose_vector[0];
  robot_traj->cal_end_point_tra_py->current_pose = current_pose_vector[1];
  robot_traj->cal_end_point_tra_pz->current_pose = current_pose_vector[2];
  robot_traj->cal_end_point_tra_alpha->current_pose = current_pose_vector[3];
  robot_traj->cal_end_point_tra_betta->current_pose = current_pose_vector[4];
  robot_traj->cal_end_point_tra_kamma->current_pose = current_pose_vector[5];
}
void TaskMotion::set_current_pose_eaa(double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  tf_current_pose_ = Transform3D<> (Vector3D<>(x, y, z), EAA<>(axis_x, axis_y, axis_z).toRotation3D());
}
void TaskMotion::clear_task_motion()
{
  task_done = false;
  all_point = -1;
  current_point = -1;
  motion_start_time_vector.clear();
  motion_task_pose_vector.clear();

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,0) =  current_pose_vector[num];
    desired_pose_matrix(num,1) =  current_pose_vector[num];
    robot_traj->current_pose_change(num,0) = current_pose_vector[num];
  }
  robot_traj->cal_end_point_tra_px->current_pose = current_pose_vector[0];
  robot_traj->cal_end_point_tra_py->current_pose = current_pose_vector[1];
  robot_traj->cal_end_point_tra_pz->current_pose = current_pose_vector[2];
  robot_traj->cal_end_point_tra_alpha->current_pose = current_pose_vector[3];
  robot_traj->cal_end_point_tra_betta->current_pose = current_pose_vector[4];
  robot_traj->cal_end_point_tra_kamma->current_pose = current_pose_vector[5];
}
void TaskMotion::clear_phase()
{
  phases_ = 0;
}
void TaskMotion::stop_motion()
{
  desired_pose_matrix.fill(0);
  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,0) = current_pose_vector[num];
    desired_pose_matrix(num,1) = current_pose_vector[num];
    desired_pose_matrix(num,7) = 1;
  }
  robot_traj->stop_trajectory();
}
void TaskMotion::set_all_phases_(int all_phases)
{
  all_phases_ = all_phases;
}
std::vector<double> TaskMotion::get_current_pose()
{
  return current_pose_vector;
}
std::vector<double> TaskMotion::get_set_point_base()
{
  return set_point_;
}
std::vector<double> TaskMotion::get_desired_force_torque()
{
  return current_force_torque_vector;
}
std::vector<double> TaskMotion::get_initial_ee_position()
{
  return initial_robot_ee_position;
}
int TaskMotion::get_phases_()
{
  return phases_;
}


