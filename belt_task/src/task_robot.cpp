/*
 * task_robot.cpp
 *
 *  Created on: Aug 19, 2020
 *      Author: yik
 */

#include "task_robot.h"
TaskRobot::TaskRobot()
{

}
TaskRobot::TaskRobot(std::string robot_name, std::string init_path)
{
  control_time_ = 0.002;
  tool_mass_ = 0;
  sub_tasks_ = 0;

  //vectors
  joint_positions_.assign(6,0); //(6);
  actual_tcp_pose_.assign(6,0); //(6);
  actual_tcp_speed_.assign(6,0); //(6);
  target_tcp_pose_.assign(6,0); //(6);
  acutal_tcp_acc_.assign(3,0); //(3);
  raw_ft_data_.assign(6,0); //(6);
  filtered_tcp_ft_data_.assign(6,0); //(6);
  contacted_ft_data_.assign(6,0); //(6);
  contacted_ft_no_offset_data_.assign(6,0); //(6);
  current_q_.assign(6,0); //(6);
  pid_compensation_.assign(6,0); //(6);

  set_point_vector_.assign(6,0); //(6);
  desired_pose_vector_.assign(6,0); //(6);
  desired_force_torque_vector_.assign(6,0); //(6);
  compensated_pose_vector_.assign(6,0); //(6);
  initial_pose_vector_.assign(6,0); //(6);
  error_ee_pose_.assign(6,0); //(6);
  compensated_q_.assign(6,0); //(6);

  limit_force_compensation_values_.assign(3,0);
  data_current_belt_.assign(3,0);
  data_desired_belt_.assign(3,0);
  data_bearing_tcp_belt_.assign(3,0);

  pulley_bearing_position_.assign(6,0);
  robot_initial_pose_.assign(6,0);

  force_controller_gain_.x_kp = 0;
  force_controller_gain_.x_ki = 0;
  force_controller_gain_.x_kd = 0;
  force_controller_gain_.y_kp = 0;
  force_controller_gain_.y_ki = 0;
  force_controller_gain_.y_kd = 0;
  force_controller_gain_.z_kp = 0;
  force_controller_gain_.z_ki = 0;
  force_controller_gain_.z_kd = 0;
  force_controller_gain_.eaa_x_kp = 0;
  force_controller_gain_.eaa_x_ki = 0;
  force_controller_gain_.eaa_x_kd = 0;
  force_controller_gain_.eaa_y_kp = 0;
  force_controller_gain_.eaa_y_ki = 0;
  force_controller_gain_.eaa_y_kd = 0;
  force_controller_gain_.eaa_z_kp = 0;
  force_controller_gain_.eaa_z_ki = 0;
  force_controller_gain_.eaa_z_kd = 0;

  position_controller_gain_.x_kp = 0;
  position_controller_gain_.x_ki = 0;
  position_controller_gain_.x_kd = 0;
  position_controller_gain_.y_kp = 0;
  position_controller_gain_.y_ki = 0;
  position_controller_gain_.y_kd = 0;
  position_controller_gain_.z_kp = 0;
  position_controller_gain_.z_ki = 0;
  position_controller_gain_.z_kd = 0;
  position_controller_gain_.eaa_x_kp = 0;
  position_controller_gain_.eaa_x_ki = 0;
  position_controller_gain_.eaa_x_kd = 0;
  position_controller_gain_.eaa_y_kp = 0;
  position_controller_gain_.eaa_y_ki = 0;
  position_controller_gain_.eaa_y_kd = 0;
  position_controller_gain_.eaa_z_kp = 0;
  position_controller_gain_.eaa_z_ki = 0;
  position_controller_gain_.eaa_z_kd = 0;

  robot_name_ = robot_name;

  //data log
  data_log_ = std::make_shared<DataLogging>(init_path + "/" + robot_name_);
  data_log_->initialize();

  //tool_estimation
  tool_estimation_  = std::make_shared<ToolEstimation>();
  tool_estimation_->initialize();

  //statistics
  statistics_math_ = std::make_shared<StatisticsMath>();

  // fifth order traj
  robot_fifth_traj_ = std::make_shared<EndEffectorTraj>();
  robot_task_ = std::make_shared<TaskMotion>();

  //setting up control time and mass of tool
  //tool_estimation_ ->set_parameters(control_time_, tool_mass_);
  robot_fifth_traj_->set_control_time(control_time_);

  robot_task_->initialize(control_time_, init_path + "/" + robot_name_ + "/initialize_robot.yaml");

  //load motion data
  robot_task_->load_task_motion(init_path + "/" + robot_name_ + "/motion/initialize.yaml", "initialize");

  desired_pose_vector_ = robot_task_ -> get_current_pose();

  //control
  force_x_compensator_ = std::make_shared<PID_function>(control_time_, 0.04, -0.04, 0, 0, 0, 0.0000001, -0.0000001, 0.5);
  force_y_compensator_ = std::make_shared<PID_function>(control_time_, 0.04, -0.04, 0, 0, 0, 0.0000001, -0.0000001, 0.5);
  force_z_compensator_ = std::make_shared<PID_function>(control_time_, 0.04, -0.04, 0, 0, 0, 0.0000001, -0.0000001, 0.5);

  position_x_controller_ = std::make_shared<PID_function>(control_time_, 0.025, -0.025, 0, 0, 0, 0.0000001, -0.0000001, 0.5);
  position_y_controller_ = std::make_shared<PID_function>(control_time_, 0.025, -0.025, 0, 0, 0, 0.0000001, -0.0000001, 0.5);
  position_z_controller_ = std::make_shared<PID_function>(control_time_, 0.025, -0.025, 0, 0, 0, 0.0000001, -0.0000001, 0.5);

  control_check_ = false;
  joint_vel_limits_ = false;
  contact_check_ = false;

  preferred_solution_number_ = 0;
  time_count_ = 0;
  previous_task_command_ = "";

  gazebo_check_ = true;

  flag = false;
  previous_phase_ = -1;
  finish_task_ = false;
  master_way_points_numbers_ = 0;
  slave_way_points_numbers_ = 0;
  robust_force_values_numbers_ = 0;

  gripper_move_values = 0;

  position_x_controller_->set_smooth_gain_time(0.5);
  position_y_controller_->set_smooth_gain_time(0.5);
  position_z_controller_->set_smooth_gain_time(0.5);
  force_x_compensator_->set_smooth_gain_time(2);
  force_y_compensator_->set_smooth_gain_time(2);
  force_z_compensator_->set_smooth_gain_time(2);

}
TaskRobot::~TaskRobot()
{

}
void TaskRobot::initialize_reference_frame(std::vector<double> temp_reference_frame_start, std::vector<double> temp_reference_frame_end)
{
  rw::math::Transform3D<> tf_base_to_tcp_;
  rw::math::Transform3D<> tf_base_to_start_;
  rw::math::Transform3D<> tf_base_to_end_;

  rw::math::Transform3D<> tf_tcp_to_start_;
  rw::math::Transform3D<> tf_tcp_to_end_;

  rw::math::Transform3D<> tf_tcp_to_direction_;
  rw::math::Transform3D<> tf_tcp_to_rotate_;

  std::vector<double> reference_frame;
  reference_frame.assign(6,0);


  rw::math::Transform3D<> temp_inv_;

  double align_angle_z_ = 0;

  tf_base_to_tcp_ = Transform3D<> (Vector3D<>(robot_initial_pose_[0], robot_initial_pose_[1], robot_initial_pose_[2]), EAA<>(robot_initial_pose_[3], robot_initial_pose_[4], robot_initial_pose_[5]).toRotation3D());
  tf_base_to_start_ = Transform3D<> (Vector3D<>(temp_reference_frame_start[0], temp_reference_frame_start[1], temp_reference_frame_start[2]), EAA<>(temp_reference_frame_start[3], temp_reference_frame_start[4], temp_reference_frame_start[5]).toRotation3D());
  tf_base_to_end_ = Transform3D<> (Vector3D<>(temp_reference_frame_end[0], temp_reference_frame_end[1], temp_reference_frame_end[2]), EAA<>(temp_reference_frame_end[3], temp_reference_frame_end[4], temp_reference_frame_end[5]).toRotation3D());


  temp_inv_ = tf_base_to_tcp_;
  temp_inv_.invMult(temp_inv_, tf_base_to_start_);
  tf_tcp_to_start_ = temp_inv_;

  temp_inv_ = tf_base_to_tcp_;
  temp_inv_.invMult(temp_inv_, tf_base_to_end_);
  tf_tcp_to_end_ = temp_inv_;

  tf_tcp_to_direction_.P() = -(tf_tcp_to_end_.P() - tf_tcp_to_start_.P());

  align_angle_z_ = atan2(tf_tcp_to_direction_.P()[1],tf_tcp_to_direction_.P()[0]);

  std::cout << robot_name_ <<"::  align_angle_z_  :: " << align_angle_z_ << std::endl;

  std::cout << robot_name_ <<"::  tf_tcp_to_direction_.P() :: " << tf_tcp_to_direction_.P() << std::endl;

  if(align_angle_z_ > 89*DEGREE2RADIAN)
    align_angle_z_ = 89*DEGREE2RADIAN;
  if(align_angle_z_ < -89*DEGREE2RADIAN)
    align_angle_z_ = -89*DEGREE2RADIAN;

//  if(!robot_name_.compare("robot_B"))
//  {
//    if(align_angle_z_ > 0)
//      align_angle_z_ = align_angle_z_ + 90*DEGREE2RADIAN;
//    if(align_angle_z_ < 0)
//      align_angle_z_ = align_angle_z_ - 90*DEGREE2RADIAN;
//    if(align_angle_z_ == 0)
//      align_angle_z_ = align_angle_z_ + 90*DEGREE2RADIAN;
//  }

  tf_tcp_to_rotate_ =  Transform3D<> (Vector3D<>(0,0,0), RPY<>(align_angle_z_,0,0).toRotation3D());

  reference_frame = temp_reference_frame_start;

  tf_base_to_start_ = tf_base_to_tcp_*tf_tcp_to_rotate_;

  reference_frame[3] = EAA<>(tf_base_to_start_.R())[0];
  reference_frame[4] = EAA<>(tf_base_to_start_.R())[1];
  reference_frame[5] = EAA<>(tf_base_to_start_.R())[2];

  std::cout << robot_name_ <<"::  reference_frame  :: " << reference_frame << std::endl;

  robot_task_->initialize_reference_frame(reference_frame);
  pulley_bearing_position_ = reference_frame;
  tf_base_to_bearing_ = Transform3D<> (Vector3D<>(pulley_bearing_position_[0], pulley_bearing_position_[1], pulley_bearing_position_[2]), EAA<>(pulley_bearing_position_[3], pulley_bearing_position_[4], pulley_bearing_position_[5]).toRotation3D());
  desired_pose_vector_ = robot_task_ -> get_current_pose();

  std::cout << robot_name_ <<"::  NEW initialize !! " << std::endl;
}
void TaskRobot::init_model(std::string wc_file, std::string robot_model)
{
  wc_ = WorkCellLoader::Factory::load(wc_file);
  device_ = wc_->findDevice<SerialDevice>(robot_model);

  if (wc_.isNull())
    RW_THROW("WorkCell could not be loaded.");
  if (device_.isNull())
    RW_THROW("UR10e device could not be found.");

  state_ = wc_->getDefaultState();
  solver_ = std::make_shared<ClosedFormIKSolverUR>(device_, state_);
  solver_->setCheckJointLimits(true);
}
void TaskRobot::initialize(std::string robot_ip, bool gazebo_check)
{
  gazebo_check_ = gazebo_check;
  if(!gazebo_check_)
  {
    //set up the robot
    rtde_receive_ = std::make_shared<RTDEReceiveInterface>(robot_ip);
    rtde_control_ = std::make_shared<RTDEControlInterface>(robot_ip);

    usleep(1000000);

    if(rtde_control_->zeroFtSensor())
    {
      usleep(1000000);
      std::cout << COLOR_GREEN_BOLD << robot_name_ << ": Completed " << "ZeroFtSensor" << COLOR_RESET << std::endl;
    }
    else
      std::cout << COLOR_RED_BOLD << robot_name_ <<": Error " << "ZeroFtSensor" << COLOR_RESET << std::endl;

    std::cout << COLOR_GREEN_BOLD << robot_name_ <<": Connected to your program" << COLOR_RESET << std::endl;
  }
}

void TaskRobot::move_to_init_pose()
{
  if(!gazebo_check_)
  {
    std::cout << COLOR_RED_BOLD << robot_name_ <<": will move 1 seconds later" << COLOR_RESET << std::endl;

    usleep(1000000);

    rtde_control_->moveL(initial_pose_vector_, 0.08, 0.08);

    std::cout << COLOR_RED_BOLD << robot_name_ <<": Send" << COLOR_RESET << std::endl;

    usleep(1000000);

    std::cout << COLOR_GREEN_BOLD << robot_name_ << ": Adjust Accelerometer Sensor and compensate gravity term" << COLOR_RESET << std::endl;
    //getting sensor values sensor filter
    acutal_tcp_acc_  = rtde_receive_->getActualToolAccelerometer();
    actual_tcp_pose_ = rtde_receive_->getActualTCPPose();

    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    tool_estimation_->set_orientation_data(tf_current_);
    tool_estimation_->set_gravity_input_data(acutal_tcp_acc_);

    //initialize
    compensated_pose_vector_ = actual_tcp_pose_;

    std::cout << robot_name_ << ": Compensated gravity terms " << acutal_tcp_acc_ << std::endl;
    std::cout << COLOR_GREEN << robot_name_ << " All of things were initialized!" << COLOR_RESET << std::endl;
  }
  else
  {
    compensated_pose_vector_ = initial_pose_vector_;
    tf_current_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]), EAA<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D());
  }
}
bool TaskRobot::tasks(std::string command) // first for only two pulleys
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
      robot_task_->set_all_phases_(master_way_points_numbers_);
      master_robot();

      if(sub_tasks_ == 1)
        finish_task_ = 1;
    }

    if(!command.compare("slave"))
    {
      robot_task_->set_all_phases_(slave_way_points_numbers_);
      slave_robot();
    }

    previous_phase_ = robot_task_->get_phases_();
    robot_task_->generate_trajectory();
    robot_task_->check_phases();
  }
  previous_task_command_ = command;

  return true;
}
void TaskRobot::master_robot()
{
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

  if(robot_task_->get_phases_() == 3)
    {
      gripper_move_values = 12;
    }

    if(robot_task_->get_phases_() == 1)
    {
      gripper_move_values = 6;

      desired_belt_[0] = master_way_points_[1][0];
      desired_belt_[1] = master_way_points_[1][1];
      desired_belt_[2] = master_way_points_[1][2];

      robot_task_->estimation_of_belt_position(desired_belt_); ////bearing frame
      robot_task_->insert_into_groove(RPY<>(master_way_points_[1][3],master_way_points_[1][4],master_way_points_[1][5]));

      return;
    }
    robot_task_->motion_to_desired_pose(contact_check_, master_way_points_[motion_phases_][0],master_way_points_[motion_phases_][1],master_way_points_[motion_phases_][2], RPY<>(master_way_points_[motion_phases_][3],master_way_points_[motion_phases_][4],master_way_points_[motion_phases_][5]),master_way_points_[motion_phases_][6]); //bearing frame
  }
}
void TaskRobot::slave_robot() // for robot A
{
  static int motion_phases_ = 0;

  if(sub_tasks_ == 0 && robot_task_->get_phases_() == slave_way_points_numbers_)
  {
    position_x_controller_->set_smooth_gain_time(5);
    position_y_controller_->set_smooth_gain_time(5);
    position_z_controller_->set_smooth_gain_time(5);
    force_x_compensator_->set_smooth_gain_time(5);
    force_y_compensator_->set_smooth_gain_time(5);
    force_z_compensator_->set_smooth_gain_time(5);

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

    robot_task_->motion_to_desired_pose(contact_check_, slave_way_points_[motion_phases_][0],slave_way_points_[motion_phases_][1],slave_way_points_[motion_phases_][2], RPY<>(slave_way_points_[motion_phases_][3],slave_way_points_[motion_phases_][4],slave_way_points_[motion_phases_][5]),slave_way_points_[motion_phases_][6]); //bearin

    for(int num = 0; num < 3; num ++)
    {
      desired_force_torque_vector_[num] = robust_force_values_[motion_phases_][num];
    }
  }
}
bool TaskRobot::hybrid_controller()
{
  //motion reference

  desired_pose_vector_ = robot_task_->get_current_pose();

  //desired_force_torque_vector_ = robot_task_->get_desired_force_torque();

  //controller for pose controller
  position_x_controller_->set_pid_gain(position_controller_gain_.x_kp,position_controller_gain_.x_ki,position_controller_gain_.x_kd);
  position_y_controller_->set_pid_gain(position_controller_gain_.x_kp,position_controller_gain_.x_ki,position_controller_gain_.x_kd);
  position_z_controller_->set_pid_gain(position_controller_gain_.x_kp,position_controller_gain_.x_ki,position_controller_gain_.x_kd);

  //controller for force compensation
  force_x_compensator_->set_pid_gain(force_controller_gain_.x_kp,force_controller_gain_.x_ki,force_controller_gain_.x_kd);
  force_y_compensator_->set_pid_gain(force_controller_gain_.x_kp,force_controller_gain_.x_ki,force_controller_gain_.x_kd);
  force_z_compensator_->set_pid_gain(force_controller_gain_.x_kp,force_controller_gain_.x_ki,force_controller_gain_.x_kd);

  if(!gazebo_check_)
  {
    //getting sensor values sensor filter
    raw_ft_data_     = rtde_receive_->getActualTCPForce();
    acutal_tcp_acc_  = rtde_receive_->getActualToolAccelerometer();
    actual_tcp_pose_ = rtde_receive_->getActualTCPPose();
    actual_tcp_speed_ = rtde_receive_->getActualTCPSpeed();
    joint_positions_ = rtde_receive_->getActualQ();
    //current_q_ = rtde_receive_->getActualQ();

    robot_task_->set_current_pose_eaa(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2],actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]);

    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    //force torque sensor filtering
    tool_estimation_->set_orientation_data(tf_current_);
    tool_estimation_->process_estimated_force(raw_ft_data_, acutal_tcp_acc_);

    contacted_ft_data_ = tool_estimation_->get_contacted_force();
    current_ft_ = Wrench6D<> (contacted_ft_data_[0], contacted_ft_data_[1], contacted_ft_data_[2], contacted_ft_data_[3], contacted_ft_data_[4], contacted_ft_data_[5]);
    current_ft_ = (tf_current_.R()).inverse()*current_ft_;
    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    position_x_controller_->PID_calculate(desired_pose_vector_[0], actual_tcp_pose_[0], 0);
    position_y_controller_->PID_calculate(desired_pose_vector_[1], actual_tcp_pose_[1], 0);
    position_z_controller_->PID_calculate(desired_pose_vector_[2], actual_tcp_pose_[2], 0);

    force_x_compensator_->PID_calculate(desired_force_torque_vector_[0],current_ft_.force()[0],0);
    force_y_compensator_->PID_calculate(desired_force_torque_vector_[1],current_ft_.force()[1],0);
    force_z_compensator_->PID_calculate(desired_force_torque_vector_[2],current_ft_.force()[2],0);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(force_x_compensator_->get_final_output(),force_y_compensator_->get_final_output(),force_z_compensator_->get_final_output()), EAA<>(0,0,0).toRotation3D());

    tf_modified_pose_ = tf_current_ * tf_tcp_desired_pose_;

    tf_modified_pose_.P() = tf_current_.P() - tf_modified_pose_.P();

    pid_compensation_[0] = position_x_controller_->get_final_output();
    pid_compensation_[1] = position_y_controller_->get_final_output();
    pid_compensation_[2] = position_z_controller_->get_final_output();
    pid_compensation_[3] = force_x_compensator_->get_final_output();
    pid_compensation_[4] = force_y_compensator_->get_final_output();
    pid_compensation_[5] = force_z_compensator_->get_final_output();

    filtered_tcp_ft_data_[0] = current_ft_.force()[0];
    filtered_tcp_ft_data_[1] = current_ft_.force()[1];
    filtered_tcp_ft_data_[2] = current_ft_.force()[2];
    filtered_tcp_ft_data_[3] = current_ft_.torque()[0];
    filtered_tcp_ft_data_[4] = current_ft_.torque()[1];
    filtered_tcp_ft_data_[5] = current_ft_.torque()[2];

    compensated_pose_vector_[0] = actual_tcp_pose_[0] + position_x_controller_->get_final_output()+(tf_modified_pose_.P())[0];
    compensated_pose_vector_[1] = actual_tcp_pose_[1] + position_y_controller_->get_final_output()+(tf_modified_pose_.P())[1];
    compensated_pose_vector_[2] = actual_tcp_pose_[2] + position_z_controller_->get_final_output()+(tf_modified_pose_.P())[2];

    compensated_pose_vector_[3] = desired_pose_vector_[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[4] = desired_pose_vector_[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[5] = desired_pose_vector_[5]; //+ force_x_compensator->get_final_output();

    //cout << compensated_pose_vector_ << endl;
  }
  else
  {

    position_x_controller_->PID_calculate(desired_pose_vector_[0], compensated_pose_vector_[0], 0);
    position_y_controller_->PID_calculate(desired_pose_vector_[1], compensated_pose_vector_[1], 0);
    position_z_controller_->PID_calculate(desired_pose_vector_[2], compensated_pose_vector_[2], 0);

    //contact_check_ = ros_state->get_test();

    pid_compensation_[0] = position_x_controller_->get_final_output();
    pid_compensation_[1] = position_y_controller_->get_final_output();
    pid_compensation_[2] = position_z_controller_->get_final_output();
    pid_compensation_[3] = 0;
    pid_compensation_[4] = 0;
    pid_compensation_[5] = 0;

    //cout << pid_compensation << endl;

    compensated_pose_vector_[0] = compensated_pose_vector_[0] + pid_compensation_[0]; //+ ros_state->get_rl_action()[0];
    compensated_pose_vector_[1] = compensated_pose_vector_[1] + pid_compensation_[1]; //+ ros_state->get_rl_action()[1];
    compensated_pose_vector_[2] = compensated_pose_vector_[2] + pid_compensation_[2]; //+ ros_state->get_rl_action()[2];

    compensated_pose_vector_[3] = desired_pose_vector_[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[4] = desired_pose_vector_[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[5] = desired_pose_vector_[5]; //+ force_x_compensator->get_final_output();


    robot_task_->set_current_pose_eaa(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2],compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]);
    tf_current_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]), EAA<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D());
  }

  error_ee_pose_[0] = position_x_controller_->get_error();
  error_ee_pose_[1] = position_y_controller_->get_error();
  error_ee_pose_[2] = position_z_controller_->get_error();
  error_ee_pose_[3] = 0; //position_x_controller->get_error();
  error_ee_pose_[4] = 0; //position_x_controller->get_error();
  error_ee_pose_[5] = 0; //position_x_controller->get_error();


  tf_desired_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]),
      EAA<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D());

  target_tcp_pose_ = compensated_pose_vector_;
  //solve ik problem
  solutions_ = solver_->solve(tf_desired_, state_);

  //for(std::size_t i = 0; i < solutions_.size(); i++) {
  //  std::cout << robot_name_ << "    :  " << i << " : " << solutions_[i] << std::endl;
  // }

  //std::cout << robot_name_  << " : " << solutions_[3] << std::endl;

  for(int num = 0; num <6 ; num ++)
  {
    compensated_q_[num] = solutions_[preferred_solution_number_].toStdVector()[num];
  }

  //from covid - robot
  for (unsigned int num = 0; num < 6; num ++)
  {
    // LOG_DEBUG("[%s]", "Wrapping for joint");
    const double diffOrig = fabs(current_q_[num] - compensated_q_[num]);
    // LOG_DEBUG("[diffOrig: %e , %e]", (solutions[i][j])/rw::math::Pi*180.0, diffOrig );

    const double diffAdd = fabs(current_q_[num] - (compensated_q_[num] + 2 * rw::math::Pi));
    // LOG_DEBUG("[diffAdd: %e , %e]", (solutions[i][j]+2*rw::math::Pi)/rw::math::Pi*180.0, diffAdd );

    const double diffSub = fabs(current_q_[num] - (compensated_q_[num] - 2 * rw::math::Pi));
    // LOG_DEBUG("[diffSub: %e , %e]", (solutions[i][j]-2*rw::math::Pi)/rw::math::Pi*180.0, diffSub );

    if (diffAdd < diffOrig && diffAdd < diffSub)
    {
      compensated_q_[num] += 2 * rw::math::Pi;
    }
    else if (diffSub < diffOrig && diffSub < diffAdd)
    {
      compensated_q_[num] -= 2 * rw::math::Pi;
    }
  }

  //std::cout << robot_name_ << "::" << compensated_q_ << "  "  << current_q_<< std::endl;

  //check velocity
  for(int num = 0; num <6 ; num ++)
  {
    if(fabs((compensated_q_[num] - current_q_[num])/control_time_) > 270*DEGREE2RADIAN)
    {
      std::cout << robot_name_ << "::" << num << "::" << fabs((compensated_q_[num] - current_q_[num])/control_time_) << std::endl;
      std::cout << COLOR_RED_BOLD << "Robot speed is so FAST" << COLOR_RESET << std::endl;
      if(!gazebo_check_)
      {
        joint_vel_limits_ = true;
        control_check_ = false;
      }
      else
        joint_vel_limits_ = false;
    }
  }

  //send command in joint space to ur robot or gazebo
  if(!joint_vel_limits_)
  {
    if(!gazebo_check_)
    {
      control_check_ = rtde_control_->servoJ(compensated_q_,0,0,control_time_,0.03,2000);
    }
    for(int num = 0; num <6 ; num ++)
    {
      current_q_[num] = compensated_q_[num];
    }
  }
  else
  {
    control_check_ = false;
  }

  time_count_ += control_time_;
  //data log save
  data_log_->set_time_count(time_count_);
  data_log_->set_data_getActualQ(joint_positions_);
  data_log_->set_data_getActualTCPPose(actual_tcp_pose_);
  data_log_->set_data_getTargetTCPPose(target_tcp_pose_);
  data_log_->set_data_getActualTCPForceTorque(raw_ft_data_);
  data_log_->set_data_getActualToolAccelerometer(acutal_tcp_acc_);
  data_log_->set_data_getFilteredTCPForceTorque(filtered_tcp_ft_data_);
  data_log_->set_data_getContactedForceTorque(contacted_ft_data_);
  data_log_->set_data_getPidCompensation(pid_compensation_);
  data_log_->set_data_getDesiredTCPPose(desired_pose_vector_);
  data_log_->set_data_getBeltPosition(data_current_belt_);
  data_log_->set_data_getDesiredBeltPosition(data_desired_belt_);  //data_bearing_tcp_belt_
  data_log_->set_data_getBearingTCPPosition(data_bearing_tcp_belt_);
  data_log_->set_data_new_line();

  return control_check_;
}
void TaskRobot::parse_init_data_(const std::string &path)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); //

  }catch(const std::exception& e) //
  {
    cout << "Fail to load yaml file!111111111" << endl;
    return;
  }

  tool_mass_= doc["tool_mass"].as<double>();
  //setting up control time and mass of tool
  tool_estimation_ ->set_parameters(control_time_, tool_mass_);

  preferred_solution_number_ =  doc["preferred_solution_number"].as<double>();
  YAML::Node initial_joint_states = doc["initial_joint_states"];
  YAML::Node pulley_bearing_position_node = doc["pulley_bearing_position"];
  YAML::Node robot_initial_pose_node = doc["robot_initial_pose"];
  YAML::Node robust_force_value_node = doc["robust_force_value"];
  YAML::Node master_pulley_node = doc["master_pulley_big"];
  YAML::Node slave_node = doc["slave_pulley_big"];

  //std::vector<double> bigger_pulley_bearing_position;
  std::vector<double> temp_points_;
  int temp_points_numbers_ = 0;

  for(int num = 0; num < 6; num ++)
  {
    pulley_bearing_position_[num] = pulley_bearing_position_node[num].as<double>();
  }

  for(int num = 0; num < 6; num ++)
  {
    robot_initial_pose_[num] = robot_initial_pose_node[num].as<double>();
  }

  initial_pose_vector_ = robot_initial_pose_;

  tf_base_to_bearing_ = Transform3D<> (Vector3D<>(pulley_bearing_position_[0], pulley_bearing_position_[1], pulley_bearing_position_[2]), EAA<>(pulley_bearing_position_[3], pulley_bearing_position_[4], pulley_bearing_position_[5]).toRotation3D());

  master_way_points_numbers_ = 0;

  for (YAML::iterator it = master_pulley_node.begin(); it != master_pulley_node.end(); ++it)
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

    //cout <<  temp_points_numbers_ <<"::"<< master_way_points_[temp_points_numbers_]  << endl;
  }

  temp_points_.clear();

  for (YAML::iterator it = robust_force_value_node.begin(); it != robust_force_value_node.end(); ++it)
  {
    robust_force_values_numbers_ ++;
    temp_points_numbers_ = it->first.as<int>();

    for(int num = 0; num < 3; num++)
    {
      temp_points_.push_back(it->second[num].as<double>());
    }

    robust_force_values_[temp_points_numbers_] = temp_points_;

    temp_points_.clear();

    //cout <<  temp_points_numbers_ <<"::"<< robust_force_values_[temp_points_numbers_]  << endl;
  }

  temp_points_.clear();

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

    //cout <<  temp_points_numbers_ <<"::"<< slave_way_points_[temp_points_numbers_]  << endl;
  }

  force_controller_gain_.x_kp = doc["f_x_kp"].as<double>();
  force_controller_gain_.x_ki = doc["f_x_ki"].as<double>();
  force_controller_gain_.x_kd = doc["f_x_kd"].as<double>();
  force_controller_gain_.y_kp = doc["f_y_kp"].as<double>();
  force_controller_gain_.y_ki = doc["f_y_ki"].as<double>();
  force_controller_gain_.y_kd = doc["f_y_kd"].as<double>();
  force_controller_gain_.z_kp = doc["f_z_kp"].as<double>();
  force_controller_gain_.z_ki = doc["f_z_ki"].as<double>();
  force_controller_gain_.z_kd = doc["f_z_kd"].as<double>();
  force_controller_gain_.eaa_x_kp = doc["f_eaa_x_kp"].as<double>();
  force_controller_gain_.eaa_x_ki = doc["f_eaa_x_ki"].as<double>();
  force_controller_gain_.eaa_x_kd = doc["f_eaa_x_kd"].as<double>();
  force_controller_gain_.eaa_y_kp = doc["f_eaa_y_kp"].as<double>();
  force_controller_gain_.eaa_y_ki = doc["f_eaa_y_ki"].as<double>();
  force_controller_gain_.eaa_y_kd = doc["f_eaa_y_kd"].as<double>();
  force_controller_gain_.eaa_z_kp = doc["f_eaa_z_kp"].as<double>();
  force_controller_gain_.eaa_z_ki = doc["f_eaa_z_ki"].as<double>();
  force_controller_gain_.eaa_z_kd = doc["f_eaa_z_kd"].as<double>();

  position_controller_gain_.x_kp = doc["p_x_kp"].as<double>();
  position_controller_gain_.x_ki = doc["p_x_ki"].as<double>();
  position_controller_gain_.x_kd = doc["p_x_kd"].as<double>();
  position_controller_gain_.y_kp = doc["p_y_kp"].as<double>();
  position_controller_gain_.y_ki = doc["p_y_ki"].as<double>();
  position_controller_gain_.y_kd = doc["p_y_kd"].as<double>();
  position_controller_gain_.z_kp = doc["p_z_kp"].as<double>();
  position_controller_gain_.z_ki = doc["p_z_ki"].as<double>();
  position_controller_gain_.z_kd = doc["p_z_kd"].as<double>();
  position_controller_gain_.eaa_x_kp = doc["p_eaa_x_kp"].as<double>();
  position_controller_gain_.eaa_x_ki = doc["p_eaa_x_ki"].as<double>();
  position_controller_gain_.eaa_x_kd = doc["p_eaa_x_kd"].as<double>();
  position_controller_gain_.eaa_y_kp = doc["p_eaa_y_kp"].as<double>();
  position_controller_gain_.eaa_y_ki = doc["p_eaa_y_ki"].as<double>();
  position_controller_gain_.eaa_y_kd = doc["p_eaa_y_kd"].as<double>();
  position_controller_gain_.eaa_z_kp = doc["p_eaa_z_kp"].as<double>();
  position_controller_gain_.eaa_z_ki = doc["p_eaa_z_ki"].as<double>();
  position_controller_gain_.eaa_z_kd = doc["p_eaa_z_kd"].as<double>();

  //for gazebo
  current_q_[0] = initial_joint_states[0].as<double>();
  current_q_[1] = initial_joint_states[1].as<double>();
  current_q_[2] = initial_joint_states[2].as<double>();

  current_q_[3] = initial_joint_states[3].as<double>();
  current_q_[4] = initial_joint_states[4].as<double>();
  current_q_[5] = initial_joint_states[5].as<double>();
}
void TaskRobot::assign_pulley(const std::string &path, std::string master, std::string slave)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); //

  }catch(const std::exception& e) //
  {
    cout << "Fail to load yaml file! assign_pulley" << endl;
    return;
  }

  YAML::Node robust_force_value_node = doc["robust_force_value"];
  YAML::Node master_pulley_node = doc[master];
  YAML::Node slave_node = doc[slave];


  std::vector<double> temp_points_;
  int temp_points_numbers_ = 0;

  master_way_points_numbers_ = 0;

  for (YAML::iterator it = master_pulley_node.begin(); it != master_pulley_node.end(); ++it)
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

    cout << robot_name_ << " " << temp_points_numbers_ <<"::"<< master_way_points_[temp_points_numbers_]  << endl;
  }

  temp_points_.clear();

  robust_force_values_numbers_ = 0;

  for (YAML::iterator it = robust_force_value_node.begin(); it != robust_force_value_node.end(); ++it)
  {
    robust_force_values_numbers_ ++;
    temp_points_numbers_ = it->first.as<int>();

    for(int num = 0; num < 3; num++)
    {
      temp_points_.push_back(it->second[num].as<double>());
    }

    robust_force_values_[temp_points_numbers_] = temp_points_;

    temp_points_.clear();

    cout << robot_name_ << " " <<  temp_points_numbers_ <<"::"<< robust_force_values_[temp_points_numbers_]  << endl;
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

    cout << robot_name_ << " " <<  temp_points_numbers_ <<"::"<< slave_way_points_[temp_points_numbers_]  << endl;
  }
}
void TaskRobot::set_force_controller_x_gain(double kp,double ki,double kd)
{
  force_controller_gain_.x_kp = kp;
  force_controller_gain_.x_ki = ki;
  force_controller_gain_.x_kd = kd;
}
void TaskRobot::set_force_controller_y_gain(double kp,double ki,double kd)
{
  force_controller_gain_.y_kp = kp;
  force_controller_gain_.y_ki = ki;
  force_controller_gain_.y_kd = kd;
}
void TaskRobot::set_force_controller_z_gain(double kp,double ki,double kd)
{
  force_controller_gain_.z_kp = kp;
  force_controller_gain_.z_ki = ki;
  force_controller_gain_.z_kd = kd;
}
void TaskRobot::set_force_controller_eaa_x_gain(double kp,double ki,double kd)
{
  force_controller_gain_.eaa_x_kp = kp;
  force_controller_gain_.eaa_x_ki = ki;
  force_controller_gain_.eaa_x_kd = kd;
}
void TaskRobot::set_force_controller_eaa_y_gain(double kp,double ki,double kd)
{
  force_controller_gain_.eaa_y_kp = kp;
  force_controller_gain_.eaa_y_ki = ki;
  force_controller_gain_.eaa_y_kd = kd;
}
void TaskRobot::set_force_controller_eaa_z_gain(double kp,double ki,double kd)
{
  force_controller_gain_.eaa_z_kp = kp;
  force_controller_gain_.eaa_z_ki = ki;
  force_controller_gain_.eaa_z_kd = kd;
}
void TaskRobot::set_position_controller_x_gain(double kp,double ki,double kd)
{
  position_controller_gain_.x_kp = kp;
  position_controller_gain_.x_ki = ki;
  position_controller_gain_.x_kd = kd;
}
void TaskRobot::set_position_controller_y_gain(double kp,double ki,double kd)
{
  position_controller_gain_.y_kp = kp;
  position_controller_gain_.y_ki = ki;
  position_controller_gain_.y_kd = kd;
}
void TaskRobot::set_position_controller_z_gain(double kp,double ki,double kd)
{
  position_controller_gain_.z_kp = kp;
  position_controller_gain_.z_ki = ki;
  position_controller_gain_.z_kd = kd;
}
void TaskRobot::set_position_controller_eaa_x_gain(double kp,double ki,double kd)
{
  position_controller_gain_.eaa_x_kp = kp;
  position_controller_gain_.eaa_x_ki = ki;
  position_controller_gain_.eaa_x_kd = kd;
}
void TaskRobot::set_position_controller_eaa_y_gain(double kp,double ki,double kd)
{
  position_controller_gain_.eaa_y_kp = kp;
  position_controller_gain_.eaa_y_ki = ki;
  position_controller_gain_.eaa_y_kd = kd;
}
void TaskRobot::set_position_controller_eaa_z_gain(double kp,double ki,double kd)
{
  position_controller_gain_.eaa_z_kp = kp;
  position_controller_gain_.eaa_z_ki = ki;
  position_controller_gain_.eaa_z_kd = kd;
}
void TaskRobot::set_tf_static_robot(rw::math::Transform3D<> tf_base_to_staric_robot, rw::math::Transform3D<> tf_base_to_bearing_static_robot)
{
  tf_base_to_static_robot_ = tf_base_to_staric_robot;
  tf_base_to_bearing_static_robot_ = tf_base_to_bearing_static_robot;
}
std::vector<double> TaskRobot::get_raw_ft_data_()
{
  return raw_ft_data_;
}
std::vector<double> TaskRobot::get_contacted_ft_data_()
{
  return filtered_tcp_ft_data_;
}
std::vector<double> TaskRobot::get_error_ee_pose_()
{
  return error_ee_pose_;
}
std::vector<double> TaskRobot::get_actual_tcp_speed_()
{
  return actual_tcp_speed_;
}
std::vector<double> TaskRobot::get_current_q_()
{
  return current_q_;
}
rw::math::Transform3D<> TaskRobot::get_tf_current_()
{
  return tf_current_;
}
rw::math::Transform3D<> TaskRobot::get_tf_base_to_bearing_()
{
  return tf_base_to_bearing_;
}
double TaskRobot::get_gripper_move_values()
{
  return  gripper_move_values;
}
void TaskRobot::terminate_robot()
{
  if(!gazebo_check_)
  {
    rtde_control_->servoStop();
    rtde_control_->stopScript();
  }
}
void TaskRobot::terminate_data_log()
{
  data_log_->save_file();
}
bool TaskRobot::get_finish_task()
{
  return finish_task_;
}
std::vector<double> TaskRobot::get_target_tcp_pose_data_()
{
  return target_tcp_pose_;
}



