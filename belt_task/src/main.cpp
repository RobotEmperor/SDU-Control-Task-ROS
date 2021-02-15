/*
 * main.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "main.h"

volatile bool exit_program = false;


void *thread_func_robot_a ( void *param )
{
  struct timespec t_next, period, t_now, t_prev, t_diff, t_all, t_all_pre;

  /* period = 2 ms * thread_id */
  period.tv_sec = 0;
  period.tv_nsec = LOOP_PERIOD; // a x ms

  std::cout << COLOR_GREEN_BOLD << "RT linux robot A start! " << COLOR_RESET << std::endl;

  if(gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot A start " << COLOR_RESET << std::endl;
  }

  current_grabs_poses[0][0] = (robot_a->get_tf_current_().P())[0];
  current_grabs_poses[0][1] = (robot_a->get_tf_current_().P())[1];
  current_grabs_poses[0][2] = (robot_a->get_tf_current_().P())[2];

  current_grabs_poses[1][0] = ((tf_a_b * robot_b->get_tf_current_()).P())[0];
  current_grabs_poses[1][1] = ((tf_a_b * robot_b->get_tf_current_()).P())[1];
  current_grabs_poses[1][2] = ((tf_a_b * robot_b->get_tf_current_()).P())[2];

  robot_a_object_estimation->set_initial_grab_poses(current_grabs_poses);

  while(!exit_program)
  {
    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all_pre = t_now;
    m.lock();

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_next = t_now;
    t_prev = t_now;

    // objects estimation

    current_grabs_poses[1][0] = (robot_a->get_tf_current_().P())[0];
    current_grabs_poses[1][1] = (robot_a->get_tf_current_().P())[1];
    current_grabs_poses[1][2] = (robot_a->get_tf_current_().P())[2];

    current_grabs_poses[0][0] = ((tf_a_b * robot_b->get_tf_current_()).P())[0];
    current_grabs_poses[0][1] = ((tf_a_b * robot_b->get_tf_current_()).P())[1];
    current_grabs_poses[0][2] = ((tf_a_b * robot_b->get_tf_current_()).P())[2];

    robot_a_object_estimation->set_current_grab_poses(current_grabs_poses);

    robot_a_object_estimation->model_estimation();

    //std::cout << robot_a_object_estimation ->get_current_object_force(1) << std::endl;

    robot_a_strategy->set_robot_current_tf(robot_a->get_tf_current_());
    robot_a_strategy->set_is_moving_check(robot_a->get_is_moving_check());

    //to do
    if(!robot_a_strategy->get_type().compare("slave"))
    {
      if(finished_insertion != 0)
      {
        //gain switch
        robot_a->set_force_controller_x_gain(0.00007,0,0);
        robot_a->set_position_controller_x_gain(0.8,0,0);
        robot_a->set_force_controller_y_gain(0.00007,0,0);
        robot_a->set_position_controller_y_gain(0.8,0,0);
        robot_a->set_force_controller_z_gain(0.00007,0,0);
        robot_a->set_position_controller_z_gain(0.8,0,0);

        robot_a_strategy->set_type("master");
      }
    }
    else
    {
      if(robot_a_strategy->get_finish_task_check())
        finished_insertion = 1;
    }

    robot_a_strategy->tasks();

    robot_a->motion_generator(robot_a_strategy->get_current_reference_frame_(), robot_a_strategy->get_current_way_points_());
    robot_a->hybrid_controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_command(robot_a->get_current_q_());
    }

    //ros_state->send_raw_ft_data(robot_a->get_target_tcp_pose_data_());
    //ros_state->send_filtered_ft_data(robot_a->get_contacted_ft_data_());

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    TIMESPEC_SUB(t_now,t_prev);
    t_diff  = t_now;

    m.unlock();

    //ros_state->send_gripper_a_move(robot_a->get_gripper_move_values());
    ros_state->update_ros_data();

    TIMESPEC_ADD (t_next, period);
    clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, NULL );

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all = t_now;

    //if(((t_all.tv_sec - t_all_pre.tv_sec) + (t_all.tv_nsec - t_all_pre.tv_nsec))*0.000001  >= 2.5)
    //  std::cout << COLOR_GREEN_BOLD << "  Elapsed time A : "<< ((t_all.tv_sec - t_all_pre.tv_sec) + (t_all.tv_nsec - t_all_pre.tv_nsec))*0.000001 << COLOR_RESET << std::endl;
  }
  return NULL;
}

void *thread_func_robot_b ( void *param )
{
  struct timespec t_next, period, t_now, t_prev, t_diff, t_all, t_all_pre;;

  /* period = 2 ms * thread_id */
  period.tv_sec = 0;
  period.tv_nsec = LOOP_PERIOD; // a x ms

  std::cout << COLOR_GREEN_BOLD << "RT linux robot B start! " << COLOR_RESET << std::endl;

  if(gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot B start " << COLOR_RESET << std::endl;
  }

  while(!exit_program)
  {
    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all_pre = t_now;

    m.lock();
    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_next = t_now;
    t_prev = t_now;

    robot_b_strategy->set_robot_current_tf(robot_b->get_tf_current_());
    robot_b_strategy->set_is_moving_check(robot_b->get_is_moving_check());

    //to do
    if(!robot_b_strategy->get_type().compare("slave"))
    {
      if(finished_insertion != 0)
      {
        //gain switch
        robot_b->set_force_controller_x_gain(0.00007,0,0);
        robot_b->set_position_controller_x_gain(0.8,0,0);
        robot_b->set_force_controller_y_gain(0.00007,0,0);
        robot_b->set_position_controller_y_gain(0.8,0,0);
        robot_b->set_force_controller_z_gain(0.00007,0,0);
        robot_b->set_position_controller_z_gain(0.8,0,0);

        robot_b_strategy->set_type("master");
      }
    }
    else
    {
      if(robot_b_strategy->get_finish_task_check())
        finished_insertion = 1;
    }

    robot_b_strategy->tasks();

    robot_b->motion_generator(robot_b_strategy->get_current_reference_frame_(), robot_b_strategy->get_current_way_points_());
    robot_b->hybrid_controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_b_command(robot_b->get_current_q_());
    }

    //ros_state->send_raw_ft_data(robot_b->get_current_q_());
    //ros_state->send_filtered_ft_data(robot_b->get_target_tcp_pose_data_());

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    TIMESPEC_SUB(t_now,t_prev);
    t_diff  = t_now;
    m.unlock();

    //ros_state->send_gripper_b_move(robot_b->get_gripper_move_values());
    ros_state->update_ros_data();

    TIMESPEC_ADD (t_next, period);
    clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, NULL );

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all = t_now;

    //if(((t_all.tv_sec - t_all_pre.tv_sec) + (t_all.tv_nsec - t_all_pre.tv_nsec))*0.000001  >= 2.01)
    //cout << COLOR_GREEN_BOLD << "  Elapsed time B : "<< ((t_all.tv_sec - t_all_pre.tv_sec) + (t_all.tv_nsec - t_all_pre.tv_nsec))*0.000001 << COLOR_RESET << endl;
  }
  return NULL;
}

void initialize()
{
  finished_insertion = 0;

  wait_command = false;
  gazebo_check = true;
  control_time = 0.002;

  robot_a_ip = "192.168.1.130";
  robot_b_ip = "192.168.1.129";

  initial_path = "/home/yik/catkin_ws/src";

  robot_path = initial_path + "/SDU-Control-Task-ROS/belt_task/config";
  robot_a = std::make_shared<TaskRobot>("robot_A",robot_path);
  robot_path = robot_path + "/wc/UR10e_2018/UR10e_a.xml";
  robot_a ->init_model(robot_path, "UR10e");

  robot_path = initial_path + "/SDU-Control-Task-ROS/belt_task/config";
  robot_b = std::make_shared<TaskRobot>("robot_B",robot_path);
  robot_path = robot_path + "/wc/UR10e_2018/UR10e_b.xml";
  robot_b ->init_model(robot_path, "UR10e");

  robot_a_strategy = std::make_shared<TaskStrategy>("robot_A");
  robot_b_strategy = std::make_shared<TaskStrategy>("robot_B");

  robot_a_strategy->initialize(initial_path + "/SDU-Control-Task-ROS/belt_task/config/robot_A/initialize_robot.yaml");
  robot_b_strategy->initialize(initial_path + "/SDU-Control-Task-ROS/belt_task/config/robot_B/initialize_robot.yaml");

  robot_a_object_estimation = std::make_shared<ObejectEstimation>();
  robot_a_object_estimation->initialize(2, initial_path + "/SDU-Control-Task-ROS/belt_task/config/objects/model_config.yaml");
  std::vector<double> temp_;
  temp_.assign(6, 0);
  //two robot
  current_grabs_poses.push_back(temp_);
  current_grabs_poses.push_back(temp_);

  tf_a_parts =  Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D());
  tf_b_parts =  Transform3D<> (Vector3D<>(-0.6654834316385497, -0.0844570012960042, 0.2551901723057327), EAA<>(-1.484318068681165, 0.6191402790204206, -0.6254296057933952 ).toRotation3D());

  tf_a_b = tf_a_parts * inverse(tf_b_parts);

  std::cout << (tf_a_parts.P())[0] << std::endl;
  std::cout << (tf_a_parts.P())[1] << std::endl;
  std::cout << (tf_a_parts.P())[2] << std::endl;

  std::cout << RPY<> (tf_a_parts.R())[0] << std::endl;
  std::cout << RPY<> (tf_a_parts.R())[1] << std::endl;
  std::cout << RPY<> (tf_a_parts.R())[2] << std::endl;

  temp_[0] = -0.739757210413974;
  temp_[1] = 0.07993900394775277;
  temp_[2] = 0.2449995438351456;

  temp_[3] = -0.7217029684216122;
  temp_[4] = -1.7591780460014375;
  temp_[5] = 1.7685571865188172;
  //
  robot_a_strategy->set_parts_data(temp_,temp_);
  //
  temp_[0] = -0.6654834316385497;
  temp_[1] = -0.0844570012960042;
  temp_[2] = 0.2551901723057327;

  temp_[3] = -1.484318068681165;
  temp_[4] =  0.6191402790204206;
  temp_[5] = -0.6254296057933952;
  //
  robot_b_strategy->set_parts_data(temp_,temp_);


}

void my_function(int sig)
{
  exit_program = true; // set flag
}


int main (int argc, char **argv)
{
  // CPU_ZERO(&cpu_robot);

  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  initialize();
  ros::init(argc, argv, "Belt_Task");
  ros::NodeHandle nh;
  ros_state = std::make_shared<RosNode>(argc,argv,"Belt_Task",nh);
  ros_state->initialize();

  signal(SIGINT, my_function);
  //  while(!wait_command)
  //  {
  //    if(exit_program)
  //    {
  //      wait_command = true;
  //      ros_state->shout_down_ros();
  //
  //      robot_a->terminate_data_log();
  //      robot_b->terminate_data_log();
  //
  //      std::cout << COLOR_RED_BOLD << "Terminate program" << COLOR_RESET << std::endl;
  //      return 0;
  //    }
  //    ros_state->update_ros_data();
  //    usleep(1);
  //  }

  while(!ros_state->check_input_paths())
  {
    if(exit_program)
    {
      wait_command = true;
      ros_state->shout_down_ros();

      robot_a->terminate_data_log();
      robot_b->terminate_data_log();

      std::cout << COLOR_RED_BOLD << "Terminate program" << COLOR_RESET << std::endl;
      return 0;
    }
    ros_state->update_ros_data();
    usleep(1);
  }

  std::cout << COLOR_YELLOW_BOLD << "Recieved :: " << ros_state->check_input_paths() << COLOR_RESET << std::endl;
  robot_a_strategy->input_paths(ros_state->get_tool_paths());


  std::cout << COLOR_YELLOW_BOLD << "Simulation On [ yes / no ]" << COLOR_RESET << std::endl;
  std::cin >> silmulation_on_off;
  //silmulation_on_off = "y";

  if(!silmulation_on_off.compare("yes") || !silmulation_on_off.compare("y"))
    std::cout << COLOR_GREEN_BOLD << "Setting up Simulation " << COLOR_RESET << std::endl;
  else
  {
    std::cout << COLOR_GREEN_BOLD << "REAL Robot, Be careful to run:" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN_BOLD << "Are you sure ? [yes / no]" << COLOR_RESET << std::endl;
    std::cin >> silmulation_on_off;
    //silmulation_on_off = "y";
    if(silmulation_on_off.compare("y")!=0)
    {
      ros_state->shout_down_ros();

      robot_a->terminate_data_log();
      robot_b->terminate_data_log();

      std::cout << COLOR_RED_BOLD << "Terminate program" << COLOR_RESET << std::endl;
      return 0 ;
    }
    gazebo_check = false;
  }
  std::cout << COLOR_GREEN_BOLD << "Program Start:" << COLOR_RESET << std::endl;

  usleep(200000);

  ros_state->update_ros_data();

  if(gazebo_check)
  {
    robot_a->moveL_to_init_pose();
    robot_b->moveL_to_init_pose();
    ros_state->send_gazebo_command(robot_a->get_current_q_());
    ros_state->send_gazebo_b_command(robot_b->get_current_q_());
  }
  else
  {
    robot_a->set_up_robot(robot_a_ip, gazebo_check);
    robot_a->moveL_to_init_pose();

    robot_b->set_up_robot(robot_b_ip, gazebo_check);
    robot_b->moveL_to_init_pose();
  }

  robot_a_strategy->set_type("master");
  robot_b_strategy->set_type("slave");



  if(!robot_a_strategy->get_type().compare("master"))
  {
    robot_a->set_force_controller_x_gain(0.00007,0,0);
    robot_a->set_position_controller_x_gain(0.8,0,0);
    robot_a->set_force_controller_y_gain(0.00007,0,0);
    robot_a->set_position_controller_y_gain(0.8,0,0);
    robot_a->set_force_controller_z_gain(0.00007,0,0);
    robot_a->set_position_controller_z_gain(0.8,0,0);
  }
  else
  {
    robot_a->set_force_controller_x_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_x_gain(0.06,0,0);
    robot_a->set_force_controller_y_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_y_gain(0.06,0,0);
    robot_a->set_force_controller_z_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_z_gain(0.06,0,0);
  }

  if(!robot_b_strategy->get_type().compare("master"))
  {
    robot_b->set_force_controller_x_gain(0.00007,0,0);
    robot_b->set_position_controller_x_gain(0.8,0,0);
    robot_b->set_force_controller_y_gain(0.00007,0,0);
    robot_b->set_position_controller_y_gain(0.8,0,0);
    robot_b->set_force_controller_z_gain(0.00007,0,0);
    robot_b->set_position_controller_z_gain(0.8,0,0);
  }
  else
  {
    robot_b->set_force_controller_x_gain(0.0008,0.000015,0);
    robot_b->set_position_controller_x_gain(0.06,0,0);
    robot_b->set_force_controller_y_gain(0.0008,0.000015,0);
    robot_b->set_position_controller_y_gain(0.06,0,0);
    robot_b->set_force_controller_z_gain(0.0008,0.000015,0);
    robot_b->set_position_controller_z_gain(0.06,0,0);
  }


  std::cout << COLOR_RED_BOLD << "ROBOT A  " << robot_a_strategy->get_type() << COLOR_RESET << std::endl;
  std::cout << COLOR_RED_BOLD << "ROBOT B  " << robot_b_strategy->get_type() << COLOR_RESET << std::endl;


  //preempt rt
  int policy;
  struct sched_param prio;
  pthread_attr_t attr_robot_a;
  pthread_t loop_robot_a;
  pthread_attr_init( &attr_robot_a);

  pthread_attr_setstacksize(&attr_robot_a, PTHREAD_STACK_MIN);

  policy = SCHED_FIFO;
  pthread_attr_setschedpolicy( &attr_robot_a, policy);

  prio.sched_priority = 90; // priority range should be btw -20 to +19
  pthread_attr_setschedparam(&attr_robot_a,&prio);
  pthread_attr_setinheritsched( &attr_robot_a, PTHREAD_EXPLICIT_SCHED);

  if ( pthread_create(&loop_robot_a, &attr_robot_a, thread_func_robot_a, (void *)(1)) ){
    perror ( "Error: pthread1_create" );
    return 1;
  }

  int policy_b;
  struct sched_param prio_b;
  pthread_attr_t attr_robot_b;
  pthread_t loop_robot_b;
  pthread_attr_init( &attr_robot_b);

  pthread_attr_setstacksize(&attr_robot_b, PTHREAD_STACK_MIN);

  policy_b = SCHED_FIFO;
  pthread_attr_setschedpolicy( &attr_robot_b, policy_b);

  prio_b.sched_priority = 89; // priority range should be btw -20 to +19
  pthread_attr_setschedparam(&attr_robot_b,&prio_b);
  pthread_attr_setinheritsched( &attr_robot_b, PTHREAD_EXPLICIT_SCHED);

  if ( pthread_create(&loop_robot_b, &attr_robot_b, thread_func_robot_b, (void *)(2)) ){
    perror ( "Error: pthread2_create" );
    return 1;
  }
  std::cout << COLOR_GREEN << "Real time task loop was created!" << COLOR_RESET << std::endl;

  while(!exit_program)
  {
    ros_state->update_ros_data();
    usleep(0.1);
  }

  /* wait for threads to finish */
  pthread_join ( loop_robot_a, NULL );
  pthread_join ( loop_robot_b, NULL );

  // terminate robot
  if(!gazebo_check)
  {
    robot_a->terminate_robot();
    robot_b->terminate_robot();
  }

  robot_a->terminate_data_log();
  robot_b->terminate_data_log();


  ros_state->shout_down_ros();

  return 0;
}
