/*
 * main.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "belt_task.h"

void loop_robot_a_proc(void *arg)
{
  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);

  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);
  //Make the task periodic with a specified loop period

  RTIME tstart_A;
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart_A = rt_timer_read();

  if(gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot A start " << COLOR_RESET << std::endl;
  }

  bool task_completed = false;
  double task_time_A = 0.0;

  robot_a->set_robust_value(-0.0515);

  while(!exit_program)
  {
    m.lock();
    ros_state->update_ros_data();
    tstart_A = rt_timer_read();

    robot_a->tasks("auto");
    robot_a->hybrid_controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_command(robot_a->get_current_q_());
    }

    ros_state->send_raw_ft_data(robot_a->get_raw_ft_data_());
    ros_state->send_filtered_ft_data(robot_a->get_contacted_ft_data_());
    //		ros_state->send_error_ee_pose(robot_a->get_error_ee_pose_());
    //		ros_state->send_ee_velocity(robot_a->get_actual_tcp_speed_());

    task_time_A = (rt_timer_read() - tstart_A)/1000000.0;
    if(task_time_A >= 2.0)
      cout << COLOR_GREEN_BOLD << "Check task's completion A : " << task_completed << " Elapsed time : "<< task_time_A << COLOR_RESET << endl;
    m.unlock();
    rt_task_wait_period(NULL);
  }
}
void loop_robot_b_proc(void *arg)
{
  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);

  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);
  //Make the task periodic with a specified loop period

  RTIME tstart_B;
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart_B = rt_timer_read();

  if(gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot B start " << COLOR_RESET << std::endl;
  }

  bool task_completed = false;
  double task_time_B = 0.0;

  robot_b->set_robust_value(0);

  while(!exit_program)
  {
    m.lock();
    //robot_b->set_tf_static_robot(robot_a->get_tf_current_(), robot_a->get_tf_base_to_bearing_());
    ros_state->update_ros_data();
    tstart_B = rt_timer_read();

    robot_b->tasks("auto");
    robot_b->hybrid_controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_b_command(robot_b->get_current_q_());
    }

    //    ros_state->send_raw_ft_data(robot_b->get_raw_ft_data_());
    //    ros_state->send_filtered_ft_data(robot_b->get_contacted_ft_data_());
    //    ros_state->send_error_ee_pose(robot_b->get_error_ee_pose_());
    //    ros_state->send_ee_velocity(robot_b->get_actual_tcp_speed_());

    task_time_B = (rt_timer_read() - tstart_B)/1000000.0;
    if(task_time_B >= 2.0)
      cout << COLOR_GREEN_BOLD << "Check task's completion B : " << task_completed << "  Elapsed time : "<< task_time_B << COLOR_RESET << endl;
    m.unlock();

    //std::cout << "robot_b->get_finish_task()!!!!!!!!" << robot_b->get_finish_task()<< std::endl;

    //done_check_ = robot_b->get_finish_task();

    rt_task_wait_period(NULL);
  }
}
void initialize()
{
  wait_command = false;
  //done_check_ = false;
  gazebo_check = true;
  control_time = 0.002;

  robot_a_ip = "192.168.1.130";
  robot_b_ip = "192.168.1.129";

  robot_path = "/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config";
  robot_a = std::make_shared<TaskRobot>("robot_A",robot_path);
  robot_path = robot_path + "/wc/UR10e_2018/UR10e_a.xml";
  robot_a ->init_model(robot_path, "UR10e");
  robot_a ->parse_init_data_("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_A/initialize_robot.yaml");

  robot_path = "/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config";
  robot_b = std::make_shared<TaskRobot>("robot_B",robot_path);
  robot_path = robot_path + "/wc/UR10e_2018/UR10e_b.xml";
  robot_b ->init_model(robot_path, "UR10e");
  robot_b ->parse_init_data_("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_B/initialize_robot.yaml");
}
void my_function(int sig)
{ // can be called asynchronously
  exit_program = true; // set flag
}

void executeAction(const belt_task::belt_task_actionGoalConstPtr &start_end, Server* as)
{
  std::cout << "program_on_!!!!!!!!" << std::endl;

  if(start_end->on_off_)
  {
    wait_command = true;

   //std::cout << "done_check_!!!!!!!!" << done_check_ << std::endl;
    while(!robot_b->get_finish_task() && !exit_program)
    {

    //m.lock();
    //done_check_ = robot_b->get_finish_task();
    //m.unlock();
    usleep(0.1);
    }
    //std::cout << "done_check_!!!!!!!!" << done_check_ << std::endl;

    //result_.sequence = 1;

    exit_program = true;

    as->setSucceeded();
  }
  else
  {
    return;
  }

  //real time task
}

int main (int argc, char **argv)
{
  signal(SIGINT, my_function);
  CPU_ZERO(&cpu_robot);

  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  initialize();
  ros::init(argc, argv, "Belt_Task");
  ros::NodeHandle nh;
  ros_state = std::make_shared<RosNode>(argc,argv,"Belt_Task",nh);
  ros_state->initialize();

  Server server(nh, "belt_task", boost::bind(&executeAction, _1, &server), false);
  server.start();

  while(!wait_command)
  {
    if(exit_program)
      wait_command = true;
    ros_state->update_ros_data();
    usleep(1);
  }

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

  //real time task
  char str[35];
  sprintf(str, "Belt Task A Start");
  rt_task_create(&loop_robot_a, str, 0, 50, 0);//Create the real time task
  rt_task_start(&loop_robot_a, &loop_robot_a_proc,0);//Since task starts in suspended mode, start task

  sprintf(str, "Belt Task B Start");
  rt_task_create(&loop_robot_b, str, 0, 51, 0);//Create the real time task
  rt_task_start(&loop_robot_b, &loop_robot_b_proc, 0);//Since task starts in suspended mode, start task
  std::cout << COLOR_GREEN << "Real time task loop was created!" << COLOR_RESET << std::endl;


  while(!exit_program)
  {
    ros_state->update_ros_data();
    usleep(0.1);
  }
  //pause();

  rt_task_delete(&loop_robot_a);
  rt_task_delete(&loop_robot_b);

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
