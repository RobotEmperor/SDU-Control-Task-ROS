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

  //vectors
  joint_positions_.assign(6,0); //(6);
  actual_tcp_pose_.assign(6,0); //(6);
  actual_tcp_speed_.assign(6,0); //(6);
  target_tcp_pose_.assign(6,0); //(6);
  acutal_tcp_acc_.assign(3,0); //(3);
  raw_ft_data_.assign(6,0); //(6);
  filtered_tcp_ft_data_.assign(6,0); //(6);
  filtered_base_ft_data_.assign(6,0); //(6);
  current_q_.assign(6,0); //(6);
  pid_compensation_.assign(6,0); //(6);

  desired_pose_vector_.assign(6,0); //(6);
  desired_force_torque_vector_.assign(6,0); //(6);
  compensated_pose_vector_.assign(6,0); //(6);
  initial_pose_vector_.assign(6,0); //(6);
  error_ee_pose_.assign(6,0); //(6);
  compensated_q_.assign(6,0); //(6);

  data_current_belt_.assign(3,0);
  data_desired_belt_.assign(3,0);

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

  //motion
  robot_motion_ = std::make_shared<TaskMotion>();

  parse_init_data_(init_path + "/" + robot_name_ + "/initialize_robot.yaml");

  robot_motion_->initialize(control_time_);
  robot_motion_->set_initial_pose(initial_pose_vector_[0], initial_pose_vector_[1], initial_pose_vector_[2], initial_pose_vector_[3], initial_pose_vector_[4], initial_pose_vector_[5]);
  desired_pose_vector_ = robot_motion_ -> get_current_pose();

  //control
  force_x_compensator_ = std::make_shared<PID_function>(control_time_, 0.04, -0.04, 0, 0, 0, 0.0000001, -0.0000001, 0.5);
  force_y_compensator_ = std::make_shared<PID_function>(control_time_, 0.04, -0.04, 0, 0, 0, 0.0000001, -0.0000001, 0.5);
  force_z_compensator_ = std::make_shared<PID_function>(control_time_, 0.04, -0.04, 0, 0, 0, 0.0000001, -0.0000001, 0.5);

  position_x_controller_ = std::make_shared<PID_function>(control_time_, 0.025, -0.025, 0, 0, 0, 0.0000001, -0.0000001, 0.5);
  position_y_controller_ = std::make_shared<PID_function>(control_time_, 0.025, -0.025, 0, 0, 0, 0.0000001, -0.0000001, 0.5);
  position_z_controller_ = std::make_shared<PID_function>(control_time_, 0.025, -0.025, 0, 0, 0, 0.0000001, -0.0000001, 0.5);

  control_check_ = false;
  joint_vel_limits_ = false;

  preferred_solution_number_ = 0;
  time_count_ = 0;

  gazebo_check_ = true;

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
void TaskRobot::set_up_robot(std::string robot_ip, bool gazebo_check)
{
  gazebo_check_ = gazebo_check;
  if(!gazebo_check_)
  {
    //set up the robot
    rtde_receive_ = std::make_shared<RTDEReceiveInterface>(robot_ip);
    rtde_control_ = std::make_shared<RTDEControlInterface>(robot_ip);

    usleep(200000);

    if(rtde_control_->zeroFtSensor())
    {
      usleep(200000);
      std::cout << COLOR_GREEN_BOLD << robot_name_ << ": Completed " << "ZeroFtSensor" << COLOR_RESET << std::endl;
    }
    else
      std::cout << COLOR_RED_BOLD << robot_name_ <<": Error " << "ZeroFtSensor" << COLOR_RESET << std::endl;

    std::cout << COLOR_GREEN_BOLD << robot_name_ <<": Connected to your program" << COLOR_RESET << std::endl;
  }
}
void TaskRobot::moveL_to_init_pose()
{
  if(!gazebo_check_)
  {
    std::cout << COLOR_RED_BOLD << robot_name_ <<": MoveL to initial pose" << COLOR_RESET << std::endl;
    rtde_control_->moveL(initial_pose_vector_, 0.08, 0.08);

    usleep(200000);

    std::cout << COLOR_GREEN_BOLD << robot_name_ << ": Adjust Accelerometer Sensor and compensate gravity term" << COLOR_RESET << std::endl;
    //getting sensor values sensor filter
    acutal_tcp_acc_  = rtde_receive_->getActualToolAccelerometer();
    actual_tcp_pose_ = rtde_receive_->getActualTCPPose();

    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    tool_estimation_->set_orientation_data(tf_current_);
    tool_estimation_->set_gravity_input_data(acutal_tcp_acc_);

    //initialize
    compensated_pose_vector_ = actual_tcp_pose_;
    std::cout << COLOR_GREEN << robot_name_ << " All of things were initialized!" << COLOR_RESET << std::endl;
  }
  else
  {
    compensated_pose_vector_ = initial_pose_vector_;
    tf_current_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]), RPY<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D());
  }
}
void TaskRobot::motion_generator(Transform3D<> temp_reference_frame, std::vector<double> way_points, std::vector<double> init_vel_ , std::vector<double> final_vel_)
{
  //std::cout << COLOR_GREEN << robot_name_ << temp_reference_frame << COLOR_RESET << std::endl;
  //std::cout << COLOR_GREEN << robot_name_ << way_points << COLOR_RESET << std::endl;
  robot_motion_->motion_to_desired_pose(temp_reference_frame, way_points[0], way_points[1], way_points[2], RPY<> (way_points[3],way_points[4],way_points[5]), way_points[6]);
  robot_motion_->add_desired_vel(init_vel_, final_vel_);
  robot_motion_->generate_fifth_order_trajectory();
}
bool TaskRobot::hybrid_controller()
{
  //motion reference

  desired_pose_vector_ = robot_motion_->get_current_pose();

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

    robot_motion_->set_current_pose_eaa(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2],actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]);

    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    //force torque sensor filtering
    tool_estimation_->set_orientation_data(tf_current_);
    tool_estimation_->process_estimated_force(raw_ft_data_, acutal_tcp_acc_);

    filtered_base_ft_data_ = tool_estimation_->get_contacted_force();
    current_ft_ = Wrench6D<> (filtered_base_ft_data_[0], filtered_base_ft_data_[1], filtered_base_ft_data_[2], filtered_base_ft_data_[3], filtered_base_ft_data_[4], filtered_base_ft_data_[5]);
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
  }
  else
  {
    position_x_controller_->PID_calculate(desired_pose_vector_[0], compensated_pose_vector_[0], 0);
    position_y_controller_->PID_calculate(desired_pose_vector_[1], compensated_pose_vector_[1], 0);
    position_z_controller_->PID_calculate(desired_pose_vector_[2], compensated_pose_vector_[2], 0);

    pid_compensation_[0] = position_x_controller_->get_final_output();
    pid_compensation_[1] = position_y_controller_->get_final_output();
    pid_compensation_[2] = position_z_controller_->get_final_output();
    pid_compensation_[3] = 0;
    pid_compensation_[4] = 0;
    pid_compensation_[5] = 0;

    compensated_pose_vector_[0] = compensated_pose_vector_[0] + pid_compensation_[0]; //+ ros_state->get_rl_action()[0];
    compensated_pose_vector_[1] = compensated_pose_vector_[1] + pid_compensation_[1]; //+ ros_state->get_rl_action()[1];
    compensated_pose_vector_[2] = compensated_pose_vector_[2] + pid_compensation_[2]; //+ ros_state->get_rl_action()[2];

    compensated_pose_vector_[3] = desired_pose_vector_[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[4] = desired_pose_vector_[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[5] = desired_pose_vector_[5]; //+ force_x_compensator->get_final_output();


    robot_motion_->set_current_pose_rpy(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2],compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]);
    tf_current_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]), RPY<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D()); // EAA
  }

  error_ee_pose_[0] = position_x_controller_->get_error();
  error_ee_pose_[1] = position_y_controller_->get_error();
  error_ee_pose_[2] = position_z_controller_->get_error();
  error_ee_pose_[3] = 0; //position_x_controller->get_error();
  error_ee_pose_[4] = 0; //position_x_controller->get_error();
  error_ee_pose_[5] = 0; //position_x_controller->get_error();


  tf_desired_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]),
      RPY<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D()); // // EAA

  target_tcp_pose_ = compensated_pose_vector_;
  //solve ik problem
  solutions_ = solver_->solve(tf_desired_, state_);

  //for(std::size_t i = 0; i < solutions_.size(); i++) {
  //  std::cout << robot_name_ << "    :  " << i << " : " << solutions_[i] << std::endl;
  // }

  //std::cout << robot_name_  << " : " << solutions_[3] << std::endl;

  const rw::math::Q confStart(6, current_q_[0], current_q_[1], current_q_[2], current_q_[3], current_q_[4], current_q_[5]);

  for (unsigned int i = 0; i < solutions_.size(); i++)
  {
    for (unsigned int j = 0; j < solutions_[i].size(); j++)
    {
      const double diffOrig = fabs(confStart[j] - solutions_[i][j]);

      const double diffAdd = fabs(confStart[j] - (solutions_[i][j] + 2 * rw::math::Pi));

      const double diffSub = fabs(confStart[j] - (solutions_[i][j] - 2 * rw::math::Pi));

      if (diffAdd < diffOrig && diffAdd < diffSub)
      {
        solutions_[i][j] += 2 * rw::math::Pi;
      }
      else if (diffSub < diffOrig && diffSub < diffAdd)
      {
        solutions_[i][j] -= 2 * rw::math::Pi;
      }
    }
  }

  rw::math::Q confBest = solutions_[preferred_solution_number_];
  for (unsigned int i = 1; i < solutions_.size(); i++)
    if ((confStart - solutions_[i]).norm2() < (confStart - confBest).norm2())
      confBest = solutions_[i];

  for(int num = 0; num <6 ; num ++)
  {
    compensated_q_[num] = confBest.toStdVector()[num];
  }

  // for initial
  //std::cout << robot_name_  << " : " << compensated_q_ << std::endl;

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
  data_log_->set_data_getContactedForceTorque(filtered_base_ft_data_);
  data_log_->set_data_getPidCompensation(pid_compensation_);
  data_log_->set_data_getDesiredTCPPose(desired_pose_vector_);
  data_log_->set_data_getBeltPosition(data_current_belt_);
  data_log_->set_data_getDesiredBeltPosition(data_desired_belt_);  //data_bearing_tcp_belt_
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
    cout << "Fail to load yaml file!" << endl;
    return;
  }
  tool_mass_= doc["tool_mass"].as<double>();
  //setting up control time and mass of tool
  tool_estimation_ ->set_parameters(control_time_, tool_mass_);

  preferred_solution_number_ =  doc["preferred_solution_number"].as<double>();
  YAML::Node initial_joint_states = doc["initial_joint_states"];
  YAML::Node robot_initial_pose = doc["robot_initial_pose"];

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

  initial_pose_vector_[0] = robot_initial_pose[0].as<double>();
  initial_pose_vector_[1] = robot_initial_pose[1].as<double>();
  initial_pose_vector_[2] = robot_initial_pose[2].as<double>();
  initial_pose_vector_[3] = robot_initial_pose[3].as<double>();
  initial_pose_vector_[4] = robot_initial_pose[4].as<double>();
  initial_pose_vector_[5] = robot_initial_pose[5].as<double>();
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
void TaskRobot::set_controller_smooth_change(double time)
{
  position_x_controller_->set_smooth_gain_time(time);
  position_y_controller_->set_smooth_gain_time(time);
  position_z_controller_->set_smooth_gain_time(time);
  force_x_compensator_->set_smooth_gain_time(time);
  force_y_compensator_->set_smooth_gain_time(time);
  force_z_compensator_->set_smooth_gain_time(time);
}
void TaskRobot::set_desired_force_values(std::vector<double> desired_force_values)
{
  desired_force_torque_vector_ = desired_force_values;
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
bool TaskRobot::get_is_moving_check()
{
  return robot_motion_->is_moving_check();
}
std::vector<double> TaskRobot::get_target_tcp_pose_data_()
{
  return target_tcp_pose_;
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




