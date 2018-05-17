/*
 * This package can plan a smooth trajectory for the case of start-to-goal (only two points specified),
 * 		and the case through multiple way-points.
 *
 * ----  Developer: Solomon Jingchun YIN, jingchun.yin@live.com
 * ----  Date:    Jan 11, 2018@ partnerX Robotics Corp., Shanghai, P.R.China;
 */

#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>

namespace trajectory_planner {

class TrajectoryPlanner {
public:
	TrajectoryPlanner();
	virtual ~TrajectoryPlanner();

	//#############################################################################################################
	/*
	 * Here starts the part for trajectory planning based on cubic-polynomial curve from start to goal;
	 */
	//#############################################################################################################
	// INPUT:
	//		--	the position of the start point and the end point;
	//		-- 	the dynamic constraints for trajectory planning: maximal velocity and maximal acceleration;
	// OUTPUT:
	// 		--  values for all the elements of the planned trajectory: the value, value_dot and value_ddot;
	bool planSmoothTrajectoryCubicPolynomialStart2Goal(float value_init,
			float value_dlt, float value_dot_max, float value_ddot_max,
			std::vector<float> & vct_value, std::vector<float> & vct_value_dot,
			std::vector<float> & vct_value_ddot,
			float time_res_sampling = 1e-2);
	//---------------------------------------------------------------------------
	// INPUT:
	//		--	the position of the start point and the end point;
	//		-- 	the dynamic constraints for trajectory planning: maximal velocity and maximal acceleration;
	// OUTPUT:
	//		-- 	the parameter for the cubic polynomial curve;
	bool specifyParametersAndTimeInterval4CubicPolynomialStart2Goal(
			float value_init, float value_dlt, float value_dot_max,
			float value_ddot_max, Eigen::Vector4f & vct_param,
			float & time_span);

	// INPUT:
	//		-- 	the parameter value associated with the related spline curve segment;
	//		--	the time incremental value for the related spline curve segment;
	// OUTPUT:
	// 		--  value for one single element on the planned trajectory: the value, value_dot and value_ddot;
	bool computeValues4CubicPolynomialStart2Goal(Eigen::Vector4f vct_params,
			float time_elapsed, float &value, float &value_dot,
			float &value_ddot);
	//---------------------------------------------------------------------------
	bool planSmoothTrajectoryProfileTrapezoidalOrTriangularStart2Goal(
			float value_init, float value_dlt, float value_dot_max,
			float value_ddot_max, std::vector<float> & vct_value,
			std::vector<float> & vct_value_dot, float time_res_sampling = 1e-2);

	//#############################################################################################################
	/*
	 * Here starts the part for trajectory planning based on piece-wise cubic-polynomial curves
	 * through a sequence of way points;
	 */
	//#############################################################################################################
	// INPUT:
	//		--	the position of all the way points;
	//		-- 	the dynamic constraints for trajectory planning: maximal velocity and maximal acceleration;
	// OUTPUT:
	// 		--  values for all the elements of the planned trajectory: the value, value_dot and value_ddot;
	bool planSmoothTrajectory4CubicSplineThroughWayPointsConsideringDynamicConstraints(
			std::vector<float> vct_waypoints, float value_dot_max,
			float value_ddot_max, std::vector<float> & vct_value,
			std::vector<float> & vct_value_dot,
			std::vector<float> & vct_value_ddot,
			float time_res_sampling = 1e-2);
	//-----------------------------------------------------------------------------------------------
	// INPUT:
	//		--	the position of all the way points;
	//		-- 	the dynamic constraints for trajectory planning: maximal velocity and maximal acceleration;
	// OUTPUT:
	// 		--  list of time intervals specified for each displacement segment based on stable motion dynamic constraints;
	bool specifyTimeIntervals4CubicSplineThroughWayPoints(
			std::vector<float> vct_waypoints, float value_dot_max,
			float value_ddot_max, std::vector<float> &vct_dlt_time,
			std::vector<float> &vct_time_elapsed);
	//-----------------------------------------------------------------------------------------------
	// INPUT:
	//		--	the position of all the way points;
	// 		--  list of time intervals specified for each displacement segment based on stable motion dynamic constraints;
	// OUTPUT:
	// 		--  the estimated parameter values for each of all the piece-wise spline curves;
	//		--	the estimated elapsed time value for each end of all the piece-wise spline curves;
	bool specifyParameters4CubicSplineThroughWayPoints(
			std::vector<float> vct_waypoints, std::vector<float> vct_dlt_time,
			Eigen::VectorXf & vct_params);

	//-----------------------------------------------------------------------------------------------
	// INPUT:
	// 		--  the estimated parameter values for each of all the piece-wise spline curves;
	//		--	the estimated time value for each of all the piece-wise spline curves;
	//		--	the time elapsed to compute the value of the planned trajectory;
	// OUTPUT:
	// 		--  value for one single element on the planned trajectory: the value, value_dot and value_ddot;
	bool planSmoothTrajectoryCubicSplineThroughWayPointsWithSpecifiedTimeIntervalsAndParameters(
			std::vector<float> vct_dlt_time,
			std::vector<float> vct_time_elapsed, Eigen::VectorXf vct_params,
			std::vector<float> & vct_value, std::vector<float> & vct_value_dot,
			std::vector<float> & vct_value_ddot,
			float time_res_sampling = 1e-2);

	//-----------------------------------------------------------------------------------------------
	// INPUT:
	// 		--  the estimated parameter values for each of all the piece-wise spline curves;
	//		--	the estimated time value for each of all the piece-wise spline curves;
	//		--	the time elapsed to compute the value of the planned trajectory;
	// OUTPUT:
	// 		--  value for one single element on the planned trajectory: the value, value_dot and value_ddot;
	bool computeValue4CubicSplineThroughWayPoints(Eigen::VectorXf vct_params,
			std::vector<float> vct_time_elapsed, float time_elapsed,
			float &value, float &value_dot, float &value_ddot);
	//--------------------------------------------------------------------------------------------
	// INPUT:
	// 		--  the estimated parameter values for each of all the piece-wise spline curves;
	//		--	the estimated elapsed time value for each end of all the piece-wise spline curves;
	//		--	the time elapsed to compute the value of the planned trajectory;
	// OUTPUT:
	//		--	the time incremental value for the related spline curve segment;
	//		-- 	the parameter value associated with the related spline curve segment;
	bool specifyParameterAndTimeIntervalWithinCubicPolynomialCurves(
			Eigen::VectorXf vct_params, std::vector<float> vct_time_elapsed,
			float time_elapsed, float &dlt_time, Eigen::Vector4f& param);

	//#############################################################################################################
	void setOutputDir(std::string dir) {
		m_output_dir = dir;
	}

	void setOutputFilename(std::string str) {
		m_output_fn = str;
	}

public:
	std::string m_output_dir;
	std::string m_output_fn;

};

} /* namespace mecanum_controller */

#endif /* TRAJECTORY_PLANNER_H_ */
