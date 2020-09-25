/*
 * main.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "belt_task.h"

volatile bool exit_program = false;


void *thread_func_robot_a ( void *param )
{
  struct timespec t_next, period, t_now, t_prev, t_diff, t_all, t_all_pre;

  /* period = 2 ms * thread_id */
  period.tv_sec = 0;
  period.tv_nsec = LOOP_PERIOD; // a x ms

  //t_next = t_now;
  //t_prev = t_now;

  std::cout << COLOR_GREEN_BOLD << "RT linux robot A start! " << COLOR_RESET << std::endl;

  if(gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot A start " << COLOR_RESET << std::endl;
  }

  while(!exit_program)
  {
    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all_pre = t_now;
    m.lock();

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_next = t_now;
    t_prev = t_now;

    //to do
    ros_state->update_ros_data();

//    if(finished_insertion == 0)
//      robot_a->tasks("slave");
//    else
//    {
//      robot_a->set_force_controller_x_gain(0.00007,0,0);
//      robot_a->set_position_controller_x_gain(0.8,0,0);
//      robot_a->set_force_controller_y_gain(0.00007,0,0);
//      robot_a->set_position_controller_y_gain(0.8,0,0);
//      robot_a->set_force_controller_z_gain(0.00007,0,0);
//      robot_a->set_position_controller_z_gain(0.8,0,0);
//
//      robot_a->tasks("master");
//    }
    robot_a->tasks("1");
    robot_a->hybrid_controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_command(robot_a->get_current_q_());
    }

    ros_state->send_raw_ft_data(robot_a->get_raw_ft_data_());
    ros_state->send_filtered_ft_data(robot_a->get_contacted_ft_data_());
    ros_state->send_gripper_a_move(robot_a->get_gripper_move_values());

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    TIMESPEC_SUB(t_now,t_prev);
    t_diff  = t_now;
    //if((t_diff.tv_sec + t_diff.tv_nsec)*0.000001 >= 2)
    //cout << COLOR_GREEN_BOLD << "  Elapsed time A : "<< (t_diff.tv_sec + t_diff.tv_nsec)*0.000001 << COLOR_RESET << endl;

    m.unlock();

    TIMESPEC_ADD (t_next, period);
    clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, NULL );

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    t_all = t_now;

    //if(((t_all.tv_sec - t_all_pre.tv_sec) + (t_all.tv_nsec - t_all_pre.tv_nsec))*0.000001  >= 2.01)
    //cout << COLOR_GREEN_BOLD << "  Elapsed time A : "<< ((t_all.tv_sec - t_all_pre.tv_sec) + (t_all.tv_nsec - t_all_pre.tv_nsec))*0.000001 << COLOR_RESET << endl;
  }
  return NULL;
}

void *thread_func_robot_b ( void *param )
{
  struct timespec t_next, period, t_now, t_prev, t_diff, t_all, t_all_pre;;

  /* period = 2 ms * thread_id */
  period.tv_sec = 0;
  period.tv_nsec = LOOP_PERIOD; // a x ms

  //t_next = t_now;
  //t_prev = t_now;

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

    //to do
    ros_state->update_ros_data();

    //robot_b->tasks("master");
    robot_b->tasks("1");
    robot_b->hybrid_controller();

    if(robot_b->get_finish_task())
      finished_insertion = 1;


    if(gazebo_check)
    {
      ros_state->send_gazebo_b_command(robot_b->get_current_q_());
    }

    //ros_state->send_gripper_b_move(robot_b->get_gripper_move_values());

    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    TIMESPEC_SUB(t_now,t_prev);
    t_diff  = t_now;
    //if((t_diff.tv_sec + t_diff.tv_nsec)*0.000001 >= 2)
    //cout << COLOR_GREEN_BOLD << "  Elapsed time B : "<< (t_diff.tv_sec + t_diff.tv_nsec)*0.000001 << COLOR_RESET << endl;

    m.unlock();

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
  reference_frame_a.assign(6,0);
  reference_frame_b.assign(6,0);

  finished_insertion = 0;

  wait_command = false;
  gazebo_check = true;
  control_time = 0.002;

  robot_a_ip = "192.168.1.130";
  robot_b_ip = "192.168.1.129";

  //example
//  reference_frame_a[0] = -0.387151602138435;
//  reference_frame_a[1] = 0.542698016520246;
//  reference_frame_a[2] = 0.366637489776256;
//  reference_frame_a[3] = -2.22142708752267;
//  reference_frame_a[4] = -2.22142706545;
//  reference_frame_a[5] = 0.0000141743083872454;

  reference_frame_b[0] = -0.551984494049336;
  reference_frame_b[1] = -0.529723677571036;
  reference_frame_b[2] = 0.346883471255243;
  reference_frame_b[3] = 2.21736419901745;
  reference_frame_b[4] = 2.21751150736451;
  reference_frame_b[5] = -0.00888003853366454;
  //

  robot_path = "/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config";
  robot_a = std::make_shared<TaskRobot>("robot_A",robot_path);
  robot_path = robot_path + "/wc/UR10e_2018/UR10e_a.xml";
  robot_a ->init_model(robot_path, "UR10e");
 // robot_a ->initialize_reference_frame(reference_frame_a);
  robot_a ->parse_init_data_("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_A/initialize_robot.yaml");

  robot_path = "/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config";
  robot_b = std::make_shared<TaskRobot>("robot_B",robot_path);
  robot_path = robot_path + "/wc/UR10e_2018/UR10e_b.xml";
  robot_b ->init_model(robot_path, "UR10e");
  robot_b ->initialize_reference_frame(reference_frame_b);
  robot_b ->parse_init_data_("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_B/initialize_robot.yaml");
}
void my_function(int sig)
{ // can be called asynchronously
  exit_program = true; // set flag

}
//void executeAction(const belt_task::belt_task_actionGoalConstPtr &start_end, Server* as)
//{
//  std::cout << "program_on_!!!!!!!!" << std::endl;
//
//  if(start_end->on_off_)
//  {
//    wait_command = true;
//
//    while(!robot_a->get_finish_task() && !exit_program)
//    {
//      usleep(0.1);
//    }
//
//    //result_.sequence = 1;
//
//    exit_program = true;
//
//    as->setSucceeded();
//  }
//  else
//  {
//    return;
//  }
//
//  //real time task
//}

int main (int argc, char **argv)
{
  // CPU_ZERO(&cpu_robot);

  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  signal(SIGINT, my_function);

  initialize();
  ros::init(argc, argv, "Belt_Task");
  ros::NodeHandle nh;
  ros_state = std::make_shared<RosNode>(argc,argv,"Belt_Task",nh);
  ros_state->initialize();

  //  Server server(nh, "belt_task", boost::bind(&executeAction, _1, &server), false);
  //  server.start();
  //
  //  while(!wait_command)
  //  {
  //    if(exit_program)
  //      wait_command = true;
  //    ros_state->update_ros_data();
  //    usleep(1);
  //  }

  std::cout << COLOR_YELLOW_BOLD << "Simulation On [ yes / no ]" << COLOR_RESET << std::endl;
  //cin >> silmulation_on_off;
  silmulation_on_off = "y";

  if(!silmulation_on_off.compare("yes") || !silmulation_on_off.compare("y"))
    std::cout << COLOR_GREEN_BOLD << "Setting up Simulation " << COLOR_RESET << std::endl;
  else
  {
    std::cout << COLOR_GREEN_BOLD << "REAL Robot, Be careful to run:" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN_BOLD << "Are you sure ? [yes / no]" << COLOR_RESET << std::endl;
    //cin >> silmulation_on_off;
    silmulation_on_off = "y";
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

  //initialize new frames

  //example
//  reference_frame_a[0] = -0.438719456804066;
//  reference_frame_a[1] = 0.270205036701779;
//  reference_frame_a[2] = 0.324309390542325;
//  reference_frame_a[3] = 2.19700957926864;
//  reference_frame_a[4] = 2.21948080204964;
//  reference_frame_a[5] = 0.0144145923312306;

  reference_frame_b[0] = -0.457459182712105;
  reference_frame_b[1] = -0.302978202982455;
  reference_frame_b[2] = 0.298919760014571;
  reference_frame_b[3] = 2.21632243861128;
  reference_frame_b[4] = 2.21940107852587;
  reference_frame_b[5] = 0.0222339502909505;


  //robot_a ->initialize_reference_frame(reference_frame_a);
  robot_b ->initialize_reference_frame(reference_frame_b);

  std::cout << COLOR_GREEN_BOLD << "Program Start:" << COLOR_RESET << std::endl;

  usleep(1000000);

  ros_state->update_ros_data();

  if(gazebo_check)
  {
    robot_a->move_to_init_pose();
    robot_b->move_to_init_pose();
    ros_state->send_gazebo_command(robot_a->get_current_q_());
    ros_state->send_gazebo_b_command(robot_b->get_current_q_());
  }
  else
  {
    robot_a->initialize(robot_a_ip, gazebo_check);
    robot_a->move_to_init_pose();

    robot_b->initialize(robot_b_ip, gazebo_check);
    robot_b->move_to_init_pose();
  }

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


  signal(SIGINT, my_function);
  while(!exit_program)
  {
    ros_state->update_ros_data();
    usleep(0.1);
  }
  //pause();

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
