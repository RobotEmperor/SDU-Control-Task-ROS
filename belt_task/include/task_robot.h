/*
 * task_robot.h
 *
 *  Created on: Aug 19, 2020
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_TASK_ROBOT_H_
#define SDU_CONTROL_TASK_ROS_BELT_TASK_INCLUDE_TASK_ROBOT_H_

#include <Eigen/Dense>
#include "log.h"

//ur_rtde library
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

//rw_robwork
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include "tool_estimation.h"
#include "sdu_math/control_function.h"
#include "sdu_math/statistics_math.h"
#include "task_motion.h"
#include "data_logging.h"

#include <unistd.h>

using namespace ur_rtde;
using namespace rw::models;
using rw::invkin::ClosedFormIKSolverUR;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;

class TaskRobot
{
public:
	TaskRobot();
	TaskRobot(std::string robot_name, std::string init_path);
	~TaskRobot();

	void parse_init_data_(const std::string &path);
	void set_up_robot(std::string robot_ip, bool gazebo_check);
	void init_model(std::string wc_file, std::string robot_model);
	void moveL_to_init_pose();

	//bool tasks(std::string command);
	bool hybrid_controller();
	void motion_generator(Transform3D<> temp_reference_frame, std::vector<double> way_points);

	void terminate_robot();
	void terminate_data_log();

	void set_force_controller_x_gain(double kp,double ki,double kd);
	void set_force_controller_y_gain(double kp,double ki,double kd);
	void set_force_controller_z_gain(double kp,double ki,double kd);

	void set_force_controller_eaa_x_gain(double kp,double ki,double kd);
	void set_force_controller_eaa_y_gain(double kp,double ki,double kd);
	void set_force_controller_eaa_z_gain(double kp,double ki,double kd);

	void set_position_controller_x_gain(double kp,double ki,double kd);
	void set_position_controller_y_gain(double kp,double ki,double kd);
	void set_position_controller_z_gain(double kp,double ki,double kd);

	void set_position_controller_eaa_x_gain(double kp,double ki,double kd);
	void set_position_controller_eaa_y_gain(double kp,double ki,double kd);
	void set_position_controller_eaa_z_gain(double kp,double ki,double kd);

	void set_controller_smooth_change(double time);
	void set_desired_force_values(std::vector<double> desired_force_values);

	std::vector<double> get_raw_ft_data_();
	std::vector<double> get_target_tcp_pose_data_();
	std::vector<double> get_contacted_ft_data_();
	std::vector<double> get_error_ee_pose_();
	std::vector<double> get_actual_tcp_speed_();
	std::vector<double> get_current_q_();
	bool get_is_moving_check();
	rw::math::Transform3D<> get_tf_current_();

private:
	struct PIDControllerGain
	{
		double x_kp;
		double x_ki;
		double x_kd;

		double y_kp;
		double y_ki;
		double y_kd;

		double z_kp;
		double z_ki;
		double z_kd;

		double eaa_x_kp;
		double eaa_x_ki;
		double eaa_x_kd;

		double eaa_y_kp;
		double eaa_y_ki;
		double eaa_y_kd;

		double eaa_z_kp;
		double eaa_z_ki;
		double eaa_z_kd;
	};

	PIDControllerGain force_controller_gain_;
	PIDControllerGain position_controller_gain_;

	State state_;

	std::string robot_name_;

	//robot interface
	std::shared_ptr<RTDEReceiveInterface> rtde_receive_;
	std::shared_ptr<RTDEControlInterface> rtde_control_;

	//traj and task motion
	std::shared_ptr<TaskMotion> robot_motion_;

	//tool estimation
	std::shared_ptr<ToolEstimation> tool_estimation_;

	//cusum_method
	std::shared_ptr<StatisticsMath> statistics_math_;

	//pid controller
	std::shared_ptr<PID_function> force_x_compensator_;
	std::shared_ptr<PID_function> force_y_compensator_;
	std::shared_ptr<PID_function> force_z_compensator_;

	std::shared_ptr<PID_function> position_x_controller_;
	std::shared_ptr<PID_function> position_y_controller_;
	std::shared_ptr<PID_function> position_z_controller_;

	//data logging
	std::shared_ptr<DataLogging> data_log_;

	//robot states
	std::vector<double> joint_positions_; //(6);
	std::vector<double> actual_tcp_pose_; //(6);
	std::vector<double> actual_tcp_speed_; //(6);
	std::vector<double> target_tcp_pose_; //(6);
	std::vector<double> acutal_tcp_acc_; //(3);
	std::vector<double> raw_ft_data_; //(6);
	std::vector<double> filtered_tcp_ft_data_; //(6);
	std::vector<double> filtered_base_ft_data_; //(6);
	std::vector<double> current_q_; //(6);
	std::vector<double> pid_compensation_; //(6);

	//control states
	std::vector<double> desired_pose_vector_; //(6);
	std::vector<double> desired_force_torque_vector_; //(6);
	std::vector<double> compensated_pose_vector_; //(6);
	std::vector<double> initial_pose_vector_; //(6);
	std::vector<double> error_ee_pose_; //(6);
	std::vector<double> compensated_q_; //(6);
  rw::math::Vector3D<> desired_belt_;

	//controller gains
	std::vector<double> force_controller_; //(6);
	std::vector<double> position_controller_; //(6);

  std::vector<double> data_current_belt_;
  std::vector<double> data_desired_belt_;

	//tf
	rw::math::Transform3D<> tf_tcp_desired_pose_;
	rw::math::Transform3D<> tf_modified_pose_;
	rw::math::Transform3D<> tf_current_;
	rw::math::Transform3D<> tf_desired_;

	rw::math::Wrench6D<> current_ft_;

	//control
	double control_time_;
	double time_count_;
	double tool_mass_;

	bool control_check_;

	//solution check
	int preferred_solution_number_;
	bool joint_vel_limits_;
	std::vector<rw::math::Q> solutions_;

	//model identification
	std::string wc_file_;
	WorkCell::Ptr wc_;
	SerialDevice::Ptr device_;
	std::shared_ptr<ClosedFormIKSolverUR> solver_;

	// gazebo
	bool gazebo_check_;
};



#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TASK_ROBOT_H_ */
