/*
 * mecanum_omni_base_controller.h
 *
 *  Created on: Jan 11, 2018
 *      Author: solomon
 */

#ifndef MECANUM_OMNI_BASE_CONTROLLER_H_
#define MECANUM_OMNI_BASE_CONTROLLER_H_

#include "mobile_base_controller.h"
#include <cmath>

namespace mecanum_controller {

class MecanumOmniBaseController: public mobile_base_controller::MobileBaseController {
public:
	MecanumOmniBaseController(float alpha_max = M_PI / 10,
			float omega_max = M_PI / 4, float a_max = 0.2, float v_max = 0.2);
	virtual ~MecanumOmniBaseController();

	//#####################################################################
	// trajectory for planar translation case of start to goal with trapezoidal or triangular velocity profile;
	bool planSmoothTrajectory4DecoupledXYCubicPolynomialStart2Goal(
			float yaw_init, Eigen::Vector2f xy_init, Eigen::Vector2f xy_goal,
			std::vector<float> &vct_x, std::vector<float> &vct_x_dot,
			std::vector<float>& vct_y, std::vector<float> &vct_y_dot);
	//--------------------------------------------------------------------------------------
	// trajectory for case of start to goal with trapezoidal or triangular velocity profile
	// in any generiSc single-variable dimension;
	bool planSmoothTrajectoryProfileGenericTranslationCubicPolynomialStart2Goal(
			Eigen::Vector2f xy_init, Eigen::Vector2f xy_goal,
			std::vector<float> &vct_dist, std::vector<float> &vct_speed);
	//--------------------------------------------------------------------------------------
	// estimate the parameters for the cubic polynomial curve;
	bool specifyParameters4GenericTranslationCubicPolynomialStart2Goal(
			Eigen::Vector2f xy_init, Eigen::Vector2f xy_goal,
			Eigen::Vector4f &param_cubic, float &time_span,
			float &angle_direction);

	bool computeValues4DecoupledTranslationXYCubicPolynomialStart2Goal(
			Eigen::Vector4f param_cubic_x, float x_init, float y_init,
			float angle_direction, float time_elapsed, float &x_ref,
			float &x_dot_ref, float &y_ref, float &y_dot_ref);

	//#####################################################################
	bool planSmoothTrajectory4DecoupledTranslationXYCubicPolynomialWayPoints(
			std::vector<float> waypoints_x, std::vector<float> waypoints_y,
			std::vector<float> &vct_x, std::vector<float> &vct_x_dot,
			std::vector<float> &vct_x_ddot, std::vector<float>& vct_y,
			std::vector<float> &vct_y_dot, std::vector<float> &vct_y_ddot);

	bool specifyTimeIntervalsAndParameters4DecoupledCubicSplineThroughWayPoints(
			std::vector<float> waypoints_x, std::vector<float> waypoints_y,
			std::vector<float> &vct_dlt_time,
			std::vector<float> &vct_time_elapsed, Eigen::VectorXf &params_x,
			Eigen::VectorXf &params_y);

	bool computeValues4DecoupledTranslationXYCubicPolynomialThroughWayPoints(
			std::vector<float> vct_time_elapsed, Eigen::VectorXf params_x,
			Eigen::VectorXf params_y, float time_gone, float &value_x,
			float &value_x_dot, float &value_x_ddot, float value_y,
			float value_y_dot, float value_y_ddot);

	//#####################################################################
	// control command to send to the omni-mobile base;
	bool applyControlStrategy(Eigen::Vector2f cmd_vel_world_ref,
			Eigen::Vector2f xy_world_ref, Eigen::Vector2f xy_world_odom,
			float yaw_odom, Eigen::Vector2f& cmd_vel_base, float lambda = 1.2);

	//  control command computation with lambda a positive number;
	// e_dot = -1 * lambda * e, e = s_r - s;
	bool computeWorldFrameCmdVel(Eigen::Vector2f cmd_vel_world_ref,
			Eigen::Vector2f xy_world_ref, Eigen::Vector2f xy_world_odom,
			float lambda, Eigen::Vector2f& cmd_vel_world);

	// project the command velocity from world frame to base frame;
	bool projectCmdFromWorld2MobileBase(float yaw,
			Eigen::Vector2f cmd_vel_world, Eigen::Vector2f& cmd_vel_base);

	//#####################################################################
	bool planSmoothTrajectoryProfileTranslationRobotCmd(float yaw_init,
			Eigen::Vector2f xy_init, Eigen::Vector2f xy_goal,
			std::vector<float> &vct_x, std::vector<float> &vct_x_dot,
			std::vector<float>& vct_y, std::vector<float> &vct_y_dot);

};

} /* namespace mecanum_controller */

#endif /* MECANUM_OMNI_BASE_CONTROLLER_H_ */
