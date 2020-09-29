/*
 * belt_task.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#ifndef SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_BELT_TASK_H_
#define SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_BELT_TASK_H_
#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT
#define LOOP_PERIOD 2e6 //Expressed in ticks // 2ms control time

#define TIMESPEC_ADD(A,B) /* A += B */ \
    do {                                   \
    (A).tv_sec += (B).tv_sec;          \
    (A).tv_nsec += (B).tv_nsec;        \
    if ( (A).tv_nsec >= 1000000000 ) { \
    (A).tv_sec++;                  \
    (A).tv_nsec -= 1000000000;     \
    }                                  \
    } while (0)

#define TIMESPEC_SUB(A,B) /* A -= B */ \
    do {                                   \
    (A).tv_sec -= (B).tv_sec;          \
    (A).tv_nsec -= (B).tv_nsec;        \
    if ( (A).tv_nsec < 0 ) {           \
    (A).tv_sec--;                  \
    (A).tv_nsec += 1000000000;     \
    }                                  \
    } while (0)



#include "log.h"
#include "ros_node.h"
#include "task_robot.h"
#include <signal.h>

//preempt rt system
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <sys/mman.h>
#include <limits.h>
#include <sched.h>


//xenomai rt system
//#include <unistd.h>
//#include <signal.h>
//#include <cstdlib>
//#include <sys/mman.h>
//#include <sys/types.h>
//#include <alchemy/task.h>
//#include <alchemy/timer.h>

//memory lock
#include <mutex>

//real time task
//RT_TASK loop_robot_a;
//RT_TASK loop_robot_b;

//cpu set to avoid overlaod
cpu_set_t cpu_robot;

void initialize();

//robot load
std::shared_ptr<TaskRobot> robot_a;
std::shared_ptr<TaskRobot> robot_b;

std::string robot_a_ip;
std::string robot_b_ip;
std::string robot_path;

std::vector<double> reference_frame_a_start;
std::vector<double> reference_frame_a_end;
std::vector<double> reference_frame_b_start;
std::vector<double> reference_frame_b_end;

std::string silmulation_on_off;
std::string selection_robot_a;
std::string selection_robot_b;
//ros
std::shared_ptr<RosNode> ros_state;
typedef actionlib::SimpleActionServer<belt_task::belt_task_actionAction> Server;
void executeAction(const belt_task::belt_task_actionGoalConstPtr &start_end, Server* as);

bool program_on_off_;
bool done_check_;
belt_task::belt_task_actionResult result_;


double control_time;
bool gazebo_check;
bool wait_command;

int finished_insertion;

std::mutex m;

#endif /* SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_BELT_TASK_H_ */
