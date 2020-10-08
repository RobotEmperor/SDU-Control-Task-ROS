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
    if(!selection_robot_a.compare("slave"))
    {
      if(finished_insertion == 0)
      {
        robot_a->tasks("slave");
      }
      else
      {
        robot_a->set_force_controller_x_gain(0.00007,0,0);
        robot_a->set_position_controller_x_gain(0.8,0,0);
        robot_a->set_force_controller_y_gain(0.00007,0,0);
        robot_a->set_position_controller_y_gain(0.8,0,0);
        robot_a->set_force_controller_z_gain(0.00007,0,0);
        robot_a->set_position_controller_z_gain(0.8,0,0);

        robot_a->tasks("master");
      }
    }
    else
    {
      robot_a->tasks("master");

      if(robot_a->get_finish_task())
        finished_insertion = 1;
    }

    robot_a->hybrid_controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_command(robot_a->get_current_q_());
    }

    ros_state->send_raw_ft_data(robot_a->get_target_tcp_pose_data_());
    //ros_state->send_filtered_ft_data(robot_a->get_contacted_ft_data_());


    clock_gettime ( CLOCK_MONOTONIC, &t_now );
    TIMESPEC_SUB(t_now,t_prev);
    t_diff  = t_now;
    //if((t_diff.tv_sec + t_diff.tv_nsec)*0.000001 >= 2)
    //cout << COLOR_GREEN_BOLD << "  Elapsed time A : "<< (t_diff.tv_sec + t_diff.tv_nsec)*0.000001 << COLOR_RESET << endl;

    m.unlock();

    ros_state->send_gripper_a_move(robot_a->get_gripper_move_values());
    ros_state->update_ros_data();

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
    if(!selection_robot_b.compare("slave"))
    {
      if(finished_insertion == 0)
      {
        robot_b->tasks("slave");
      }
      else
      {
        robot_b->set_force_controller_x_gain(0.00007,0,0);
        robot_b->set_position_controller_x_gain(0.8,0,0);
        robot_b->set_force_controller_y_gain(0.00007,0,0);
        robot_b->set_position_controller_y_gain(0.8,0,0);
        robot_b->set_force_controller_z_gain(0.00007,0,0);
        robot_b->set_position_controller_z_gain(0.8,0,0);

        robot_b->tasks("master");
      }
    }
    else
    {
      robot_b->tasks("master");

      if(robot_b->get_finish_task())
        finished_insertion = 1;
    }

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
    //if((t_diff.tv_sec + t_diff.tv_nsec)*0.000001 >= 2)
    //cout << COLOR_GREEN_BOLD << "  Elapsed time B : "<< (t_diff.tv_sec + t_diff.tv_nsec)*0.000001 << COLOR_RESET << endl;

    m.unlock();

    ros_state->send_gripper_b_move(robot_b->get_gripper_move_values());
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
  reference_frame_a_start.assign(6,0);
  reference_frame_a_end.assign(6,0);
  reference_frame_b_start.assign(6,0);
  reference_frame_b_end.assign(6,0);

  finished_insertion = 0;

  wait_command = false;
  gazebo_check = true;
  control_time = 0.002;

  robot_a_ip = "192.168.1.130";
  robot_b_ip = "192.168.1.129";

  selection_robot_a = "";
  selection_robot_b = "";

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
  //robot_b ->initialize_reference_frame(reference_frame_b);
  robot_b ->parse_init_data_("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_B/initialize_robot.yaml");
}
void my_function(int sig)
{ // can be called asynchronously
  exit_program = true; // set flag

}

void executeAction(const belt_task::belt_task_actionGoalConstPtr &start_end, Server* as)
{
  double robot_big_pulley = 0;
  double robot_small_pulley = 0;
  double temp_big_x = 0;
  double temp_big_y = 0;
  double temp_big_z = 0;

  double temp_small_x = 0;
  double temp_small_y = 0;
  double temp_small_z = 0;

  std::cout << "program_on_!!!!!!!!" << std::endl;

  std::cout << "transform_a_big_pulley   :  " << start_end->transform_a_big_pulley << std::endl;
  std::cout << "transform_a_small_pulley   :  " << start_end->transform_a_small_pulley << std::endl;
  std::cout << "transform_b_big_pulley   :  " << start_end->transform_b_big_pulley << std::endl;
  std::cout << "transform_b_small_pulley   :  " << start_end->transform_b_small_pulley << std::endl;

//  transform_a_big_pulley   :  Q[6]{-0.57389, 0.376311, 0.0549096, -1.58493, 0.914809, 1.57408}
//  transform_a_small_pulley   :  Q[6]{-0.459831, 0.444951, 0.0550761, -1.09482, 1.27564, 1.08678}
//  transform_b_big_pulley   :  Q[6]{-0.836103, -0.384349, 0.0623581, 0.823434, 1.40879, -0.828878}
//  transform_b_small_pulley   :  Q[6]{-0.950775, -0.451959, 0.0622844, 1.32863, 1.1246, -1.33446}


  if(start_end->on_off)
  {
    wait_command = true;

    temp_big_x = start_end->transform_a_big_pulley[0];
    temp_big_y = start_end->transform_a_big_pulley[1];
    temp_big_z = start_end->transform_a_big_pulley[2];

    temp_small_x = start_end->transform_a_small_pulley[0];
    temp_small_y = start_end->transform_a_small_pulley[1];
    temp_small_z = start_end->transform_a_small_pulley[2];

    //    temp_big_x = -0.57389;
    //    temp_big_y = 0.376311;
    //    temp_big_z = 0.0549096;
    //
    //    temp_small_x = -0.459831;
    //    temp_small_y = 0.444951;
    //    temp_small_z = 0.0550761;

    robot_big_pulley = sqrt(pow(temp_big_x,2) + pow(temp_big_y,2) + pow(temp_big_z,2));
    robot_small_pulley = sqrt(pow(temp_small_x,2) + pow(temp_small_y,2) + pow(temp_small_z,2));

    std::cout << "robot_big_pulley   :  " << robot_big_pulley << std::endl;
    std::cout << "robot_small_pulley   :  " << robot_small_pulley << std::endl;

    if( temp_big_y > temp_small_y)
    {
      selection_robot_a = "slave"; // small
      selection_robot_b = "master"; // big
    }
    if(temp_big_y < temp_small_y)
    {
      selection_robot_a = "master";// big
      selection_robot_b = "slave";// small
    }
    //
    //  if(fabs(temp_big_y) == fabs(temp_small_y) && fabs(temp_big_x) == fabs(temp_small_x))
    //  {
    //    selection_robot_a = "slave";// small
    //    selection_robot_b = "master";//big
    //  }


    if(!selection_robot_a.compare("master") && !selection_robot_b.compare("slave"))
    {
      reference_frame_a_start[0] = start_end->transform_a_big_pulley[0];
      reference_frame_a_start[1] = start_end->transform_a_big_pulley[1];
      reference_frame_a_start[2] = start_end->transform_a_big_pulley[2];

      reference_frame_a_end[0] = start_end->transform_a_small_pulley[0];
      reference_frame_a_end[1] = start_end->transform_a_small_pulley[1];
      reference_frame_a_end[2] = start_end->transform_a_small_pulley[2];


      reference_frame_b_start[0] = start_end->transform_b_small_pulley[0];
      reference_frame_b_start[1] = start_end->transform_b_small_pulley[1];
      reference_frame_b_start[2] = start_end->transform_b_small_pulley[2];

      reference_frame_b_end[0] = start_end->transform_b_big_pulley[0];
      reference_frame_b_end[1] = start_end->transform_b_big_pulley[1];
      reference_frame_b_end[2] = start_end->transform_b_big_pulley[2];
    }

    if(!selection_robot_a.compare("slave") && !selection_robot_b.compare("master"))
    {
      reference_frame_a_start[0] = start_end->transform_a_small_pulley[0];
      reference_frame_a_start[1] = start_end->transform_a_small_pulley[1];
      reference_frame_a_start[2] = start_end->transform_a_small_pulley[2];

      reference_frame_a_end[0] = start_end->transform_a_big_pulley[0];
      reference_frame_a_end[1] = start_end->transform_a_big_pulley[1];
      reference_frame_a_end[2] = start_end->transform_a_big_pulley[2];

      reference_frame_b_start[0] = start_end->transform_b_big_pulley[0];
      reference_frame_b_start[1] = start_end->transform_b_big_pulley[1];
      reference_frame_b_start[2] = start_end->transform_b_big_pulley[2];

      reference_frame_b_end[0] = start_end->transform_b_small_pulley[0];
      reference_frame_b_end[1] = start_end->transform_b_small_pulley[1];
      reference_frame_b_end[2] = start_end->transform_b_small_pulley[2];


    }

    //    reference_frame_a_start[0] = 0;
    //    reference_frame_a_start[1] = 0;
    //    reference_frame_a_start[2] = 0;

    reference_frame_a_start[3] = 0;
    reference_frame_a_start[4] = 0;
    reference_frame_a_start[5] = 0;

    //    reference_frame_a_end[0] = 0;
    //    reference_frame_a_end[1] = 0;
    //    reference_frame_a_end[2] = 0;

    reference_frame_a_end[3] = 0;
    reference_frame_a_end[4] = 0;
    reference_frame_a_end[5] = 0;


    //    reference_frame_b_start[0] = 0;
    //    reference_frame_b_start[1] = 0;
    //    reference_frame_b_start[2] = 0;

    reference_frame_b_start[3] = 0;
    reference_frame_b_start[4] = 0;
    reference_frame_b_start[5] = 0;

    //    reference_frame_b_end[0] = 0;
    //    reference_frame_b_end[1] = 0;
    //    reference_frame_b_end[2] = 0;

    reference_frame_b_end[3] = 0;
    reference_frame_b_end[4] = 0;
    reference_frame_b_end[5] = 0;

    //    robot_a ->initialize_reference_frame(reference_frame_a_start,reference_frame_a_end);
    //    robot_b ->initialize_reference_frame(reference_frame_b_start,reference_frame_b_end);

    if(!selection_robot_a.compare("slave") && !selection_robot_b.compare("master"))
    {
      while(!robot_a->get_finish_task() && !exit_program)
      {
        usleep(0.1);
      }

    }
    else
    {
      while(!robot_b->get_finish_task() && !exit_program)
      {
        usleep(0.1);
      }

    }

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
  // CPU_ZERO(&cpu_robot);

  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  signal(SIGINT, my_function);

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
  cin >> silmulation_on_off;
  //silmulation_on_off = "y";

  if(!silmulation_on_off.compare("yes") || !silmulation_on_off.compare("y"))
    std::cout << COLOR_GREEN_BOLD << "Setting up Simulation " << COLOR_RESET << std::endl;
  else
  {
    std::cout << COLOR_GREEN_BOLD << "REAL Robot, Be careful to run:" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN_BOLD << "Are you sure ? [yes / no]" << COLOR_RESET << std::endl;
    cin >> silmulation_on_off;
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

  std::cout << "program_on_!!!!!!!!" << std::endl;

//  double robot_big_pulley = 0;
//  double robot_small_pulley = 0;
//  double temp_big_x = 0;
//  double temp_big_y = 0;
//  double temp_big_z = 0;
//
//  double temp_small_x = 0;
//  double temp_small_y = 0;
//  double temp_small_z = 0;
//
//  std::cout << "program_on_!!!!!!!!" << std::endl;
//
//
//
//  //  transform_a_big_pulley   :  Q[6]{-0.57389, 0.376311, 0.0549096, -1.58493, 0.914809, 1.57408}
//  //  transform_a_small_pulley   :  Q[6]{-0.459831, 0.444951, 0.0550761, -1.09482, 1.27564, 1.08678}
//  //  transform_b_big_pulley   :  Q[6]{-0.836103, -0.384349, 0.0623581, 0.823434, 1.40879, -0.828878}
//  //  transform_b_small_pulley   :  Q[6]{-0.950775, -0.451959, 0.0622844, 1.32863, 1.1246, -1.33446}
//
//
//  temp_big_x = -0.57389;
//  temp_big_y = 0.376311;
//  temp_big_z = 0.0549096;
//
//  temp_small_x = -0.459831;
//  temp_small_y = 0.444951;
//  temp_small_z = 0.0550761;
//
//  //    -0.836103
//  //    -0.384349
//  //    0.0623581
//  //
//  //    -0.950775
//  //    -0.451959
//  //    0.0622844
//
//  robot_big_pulley = sqrt(pow(temp_big_x,2) + pow(temp_big_y,2) + pow(temp_big_z,2));
//  robot_small_pulley = sqrt(pow(temp_small_x,2) + pow(temp_small_y,2) + pow(temp_small_z,2));
//
//  std::cout << "robot_big_pulley   :  " << robot_big_pulley << std::endl;
//  std::cout << "robot_small_pulley   :  " << robot_small_pulley << std::endl;
//
//  if( temp_big_y > temp_small_y)
//  {
//    selection_robot_a = "slave"; // small
//    selection_robot_b = "master"; // big
//  }
//  if(temp_big_y < temp_small_y)
//  {
//    selection_robot_a = "master";// big
//    selection_robot_b = "slave";// small
//  }
//  //
//  //  if(fabs(temp_big_y) == fabs(temp_small_y) && fabs(temp_big_x) == fabs(temp_small_x))
//  //  {
//  //    selection_robot_a = "slave";// small
//  //    selection_robot_b = "master";//big
//  //  }
//
//
//  if(!selection_robot_a.compare("master") && !selection_robot_b.compare("slave"))
//  {
//    reference_frame_a_start[0] = -0.57389;
//    reference_frame_a_start[1] = 0.376311;
//    reference_frame_a_start[2] = 0.0549096;
//
//    reference_frame_a_end[0] = -0.459831;
//    reference_frame_a_end[1] = 0.444951;
//    reference_frame_a_end[2] = 0.0550761;
//
//
//    reference_frame_b_start[0] = -0.950775;
//    reference_frame_b_start[1] = -0.451959;
//    reference_frame_b_start[2] = 0.0622844;
//
//    reference_frame_b_end[0] = -0.836103;
//    reference_frame_b_end[1] = -0.384349;
//    reference_frame_b_end[2] = 0.0623581;
//  }
//
//  if(!selection_robot_a.compare("slave") && !selection_robot_b.compare("master"))
//  {
//    reference_frame_a_start[0] =  -0.459831;
//    reference_frame_a_start[1] =  0.444951;
//    reference_frame_a_start[2] =  0.0550761;
//
//    reference_frame_a_end[0] = -0.57389;
//    reference_frame_a_end[1] = 0.376311;
//    reference_frame_a_end[2] = 0.0549096;
//
//    reference_frame_b_start[0] = -0.836103;
//    reference_frame_b_start[1] = -0.384349;
//    reference_frame_b_start[2] = 0.0623581;
//
//    reference_frame_b_end[0] = -0.950775;
//    reference_frame_b_end[1] = -0.451959;
//    reference_frame_b_end[2] = 0.0622844;
//
//
//  }
//
//  //    reference_frame_a_start[0] = 0;
//  //    reference_frame_a_start[1] = 0;
//  //    reference_frame_a_start[2] = 0;
//
//  reference_frame_a_start[3] = 0;
//  reference_frame_a_start[4] = 0;
//  reference_frame_a_start[5] = 0;
//
//  //    reference_frame_a_end[0] = 0;
//  //    reference_frame_a_end[1] = 0;
//  //    reference_frame_a_end[2] = 0;
//
//  reference_frame_a_end[3] = 0;
//  reference_frame_a_end[4] = 0;
//  reference_frame_a_end[5] = 0;
//
//
//  //    reference_frame_b_start[0] = 0;
//  //    reference_frame_b_start[1] = 0;
//  //    reference_frame_b_start[2] = 0;
//
//  reference_frame_b_start[3] = 0;
//  reference_frame_b_start[4] = 0;
//  reference_frame_b_start[5] = 0;
//
//  //    reference_frame_b_end[0] = 0;
//  //    reference_frame_b_end[1] = 0;
//  //    reference_frame_b_end[2] = 0;
//
//  reference_frame_b_end[3] = 0;
//  reference_frame_b_end[4] = 0;
//  reference_frame_b_end[5] = 0;


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

  robot_a ->initialize_reference_frame(reference_frame_a_start,reference_frame_a_end,selection_robot_a);
  robot_b ->initialize_reference_frame(reference_frame_b_start,reference_frame_b_end,selection_robot_b);

  std::string go_stop;
  cin >> go_stop;
  //silmulation_on_off = "y";
  if(go_stop.compare("y")!=0)
  {
    ros_state->shout_down_ros();
    robot_a->terminate_data_log();
    robot_b->terminate_data_log();

    std::cout << COLOR_RED_BOLD << "Terminate program" << COLOR_RESET << std::endl;
    return 0 ;
  }

  std::cout << COLOR_RED_BOLD << "ROBOT A  " << selection_robot_a << COLOR_RESET << std::endl;
  std::cout << COLOR_RED_BOLD << "ROBOT B  " << selection_robot_b << COLOR_RESET << std::endl;


  if(!selection_robot_a.compare("master"))
  {
    robot_a->set_force_controller_x_gain(0.00007,0,0);
    robot_a->set_position_controller_x_gain(0.8,0,0);
    robot_a->set_force_controller_y_gain(0.00007,0,0);
    robot_a->set_position_controller_y_gain(0.8,0,0);
    robot_a->set_force_controller_z_gain(0.00007,0,0);
    robot_a->set_position_controller_z_gain(0.8,0,0);
    robot_a ->assign_pulley("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_A/initialize_robot.yaml", "master_pulley_big", "slave_pulley_big");
  }
  else
  {
    robot_a->set_force_controller_x_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_x_gain(0.06,0,0);
    robot_a->set_force_controller_y_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_y_gain(0.06,0,0);
    robot_a->set_force_controller_z_gain(0.0008,0.000015,0);
    robot_a->set_position_controller_z_gain(0.06,0,0);
    robot_a ->assign_pulley("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_A/initialize_robot.yaml", "master_pulley_small", "slave_pulley_small");
  }

  if(!selection_robot_b.compare("master"))
  {
    robot_b->set_force_controller_x_gain(0.00007,0,0);
    robot_b->set_position_controller_x_gain(0.8,0,0);
    robot_b->set_force_controller_y_gain(0.00007,0,0);
    robot_b->set_position_controller_y_gain(0.8,0,0);
    robot_b->set_force_controller_z_gain(0.00007,0,0);
    robot_b->set_position_controller_z_gain(0.8,0,0);
    robot_b ->assign_pulley("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_B/initialize_robot.yaml", "master_pulley_big", "slave_pulley_big");
  }
  else
  {
    robot_b->set_force_controller_x_gain(0.0008,0.000015,0);
    robot_b->set_position_controller_x_gain(0.06,0,0);
    robot_b->set_force_controller_y_gain(0.0008,0.000015,0);
    robot_b->set_position_controller_y_gain(0.06,0,0);
    robot_b->set_force_controller_z_gain(0.0008,0.000015,0);
    robot_b->set_position_controller_z_gain(0.06,0,0);
    robot_b ->assign_pulley("/home/yik/catkin_ws/src/SDU-Control-Task-ROS/belt_task/config/robot_B/initialize_robot.yaml", "master_pulley_small", "slave_pulley_small");
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
