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

  robot_a_object_estimation->set_initial_grab_poses(robot_a->get_tf_current_());

  robot_a->set_tf_part_base(robot_a_strategy->get_a_part_frame_());

  while(!exit_program)
  {
    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all_pre = t_now;
    m.lock();

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_next = t_now;
    t_prev = t_now;

    // objects estimation
    current_grabs_poses[0][0] = (robot_a->get_tf_current_().P())[0];
    current_grabs_poses[0][1] = (robot_a->get_tf_current_().P())[1];
    current_grabs_poses[0][2] = (robot_a->get_tf_current_().P())[2];


    //robot_a_object_estimation->set_current_grab_poses(robot_a->get_tf_current_());

    //robot_a_object_estimation->model_estimation();


    robot_a_strategy->set_robot_current_tf(robot_a->get_tf_current_());
    robot_a_strategy->set_is_moving_check(robot_a->get_is_moving_check());

    //to do
    if(!robot_a_strategy->get_type().compare("slave"))
    {
      if(finished_insertion != 0)
      {
        //gain switch
//        robot_a->set_force_controller_x_gain(0.00007,0,0);
//        robot_a->set_position_controller_x_gain(0.8,0,0);
//        robot_a->set_force_controller_y_gain(0.00007,0,0);
//        robot_a->set_position_controller_y_gain(0.8,0,0);
//        robot_a->set_force_controller_z_gain(0.00007,0,0);
//        robot_a->set_position_controller_z_gain(0.8,0,0);

        robot_a->set_force_controller_x_gain(0.0008,0.000015,0);
        robot_a->set_position_controller_x_gain(0.06,0,0);
        robot_a->set_force_controller_y_gain(0.0008,0.000015,0);
        robot_a->set_position_controller_y_gain(0.06,0,0);
        robot_a->set_force_controller_z_gain(0.0008,0.000015,0);
        robot_a->set_position_controller_z_gain(0.06,0,0);

        robot_a_strategy->set_type("master");
      }
    }
    else
    {
      if(robot_a_strategy->get_finish_task_check())
      {
        exit_program = true;
        finished_insertion = 1;
      }
    }

    robot_a_strategy->tasks();

    robot_a->motion_generator(robot_a_strategy->get_current_reference_frame_(), robot_a_strategy->get_current_way_points_(),robot_a_strategy->get_current_way_init_vel_points_(),robot_a_strategy->get_current_way_final_vel_points_());
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

    ros_state->send_gripper_a_move(robot_a_object_estimation->get_current_object_force());
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

    robot_b->motion_generator(robot_b_strategy->get_current_reference_frame_(), robot_b_strategy->get_current_way_points_(), robot_b_strategy->get_current_way_init_vel_points_(), robot_b_strategy->get_current_way_final_vel_points_());
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

  //two robot grab poses
  std::vector<double> temp_;
  temp_.assign(6, 0);
  current_grabs_poses.push_back(temp_);
  current_grabs_poses.push_back(temp_);
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

  std::cout << COLOR_YELLOW_BOLD << "Waiting for input paths :: "<< COLOR_RESET << std::endl;

  //trajopt experiments
//  while(!ros_state->check_input_paths())
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

  std::cout << COLOR_YELLOW_BOLD << "Recieved :: " << ros_state->check_input_paths() << COLOR_RESET << std::endl;
  robot_a_strategy->input_paths(ros_state->get_a_tool_paths());
  robot_b_strategy->input_b_paths(ros_state->get_b_tool_paths());

  Transform3D<> initial_position_a_;
  Transform3D<> initial_position_b_;
  Transform3D<> tf_a_big_pulley;
  Transform3D<> tf_b_big_pulley;
  Transform3D<> tf_a_b;

  Transform3D<> tf_world_base_a;
  Transform3D<> tf_world_base_b;

  Transform3D<> tf_cell_base_a;
  Transform3D<> tf_cell_base_b;

  //Transform3D<> tf_a_base_corner;
  //Transform3D<> tf_corner_to_big_pulley;

  tf_cell_base_a = Transform3D<> (Vector3D<>(0.596718340385182, -0.308285332939805, 0.00557538063819106), EAA<>(0.00969272790958452, 0.0051652521972913, -1.5820114865231).toRotation3D());
  tf_cell_base_b = Transform3D<> (Vector3D<>(0.5967482693965416, 1.0980636814842386, 0.009255715558339806), EAA<>(0.00460145682828943, -0.003686739825168772, 1.5685774911729364).toRotation3D());
  //tf_a_base_corner = Transform3D<> (Vector3D<>(-0.7734, 0.1032, 0.2001), EAA<>(-0.7313354979535918, -1.7622417854008892, 1.7568650899341813).toRotation3D());
  //tf_corner_to_big_pulley = Transform3D<> (Vector3D<>(-0.04, -0.0425, 0), EAA<>(0, 0, 0).toRotation3D());
  //tf_a_big_pulley = tf_a_base_corner*tf_corner_to_big_pulley;

  tf_world_base_a = Transform3D<> (Vector3D<>(0, 0, 0.05), EAA<>(0, 0, 180*DEGREE2RADIAN).toRotation3D());

  tf_a_big_pulley = robot_a_strategy->get_a_part_frame_();
  tf_b_big_pulley = robot_a_strategy->get_b_part_frame_();

  tf_a_b = inverse(tf_cell_base_a)*tf_cell_base_b;
  //tf_b_big_pulley = inverse(tf_a_b) * tf_a_big_pulley;

  initial_position_a_ = Transform3D<> (Vector3D<>(0, 0, 0), RPY<> (0,0,0).toRotation3D()); // RPY
  initial_position_a_ = tf_a_big_pulley * initial_position_a_ ;

  initial_position_b_ = Transform3D<> (Vector3D<>(0.02, 0, -0.04), RPY<> (0,-25*DEGREE2RADIAN,0).toRotation3D()); // RPY
  initial_position_b_ = tf_b_big_pulley * initial_position_b_ ;


  tf_world_base_b = tf_world_base_a * tf_a_b;




  std::cout << "A :: ---------------------------- " << std::endl;

  std::cout << (initial_position_a_.P())[0] << std::endl;
  std::cout << (initial_position_a_.P())[1] << std::endl;
  std::cout << (initial_position_a_.P())[2] << std::endl;

  std::cout << EAA<> (initial_position_a_.R())[0] << std::endl;
  std::cout << EAA<> (initial_position_a_.R())[1] << std::endl;
  std::cout << EAA<> (initial_position_a_.R())[2] << std::endl;

  std::cout << "B :: ---------------------------- " << std::endl;

  std::cout << (initial_position_b_.P())[0] << std::endl;
  std::cout << (initial_position_b_.P())[1] << std::endl;
  std::cout << (initial_position_b_.P())[2] << std::endl;

  std::cout << EAA<> (initial_position_b_.R())[0] << std::endl;
  std::cout << EAA<> (initial_position_b_.R())[1] << std::endl;
  std::cout << EAA<> (initial_position_b_.R())[2] << std::endl;

  std::cout << "a_b :: ---------------------------- " << std::endl;

//  std::cout << RPY<> (tf_cell_base_b.R())[0] << std::endl;
//  std::cout << RPY<> (tf_cell_base_b.R())[1] << std::endl;
//  std::cout << RPY<> (tf_cell_base_b.R())[2] << std::endl;
//
//  std::cout << RPY<> (tf_cell_base_a.R())[0] << std::endl;
//  std::cout << RPY<> (tf_cell_base_a.R())[1] << std::endl;
//  std::cout << RPY<> (tf_cell_base_a.R())[2] << std::endl;

  //exit_program = true;


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
//    robot_a->set_force_controller_x_gain(0.00007,0,0);
//    robot_a->set_position_controller_x_gain(0.8,0,0);
//    robot_a->set_force_controller_y_gain(0.00007,0,0);
//    robot_a->set_position_controller_y_gain(0.8,0,0);
//    robot_a->set_force_controller_z_gain(0.00007,0,0);
//    robot_a->set_position_controller_z_gain(0.8,0,0);

    robot_a->set_force_controller_x_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_x_gain(0.06,0,0);
    robot_a->set_force_controller_y_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_y_gain(0.06,0,0);
    robot_a->set_force_controller_z_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_z_gain(0.06,0,0);
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
