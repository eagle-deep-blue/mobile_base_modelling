/*
 * mobile_base_controller.h
 *
 *  Created on: Jan 8, 2018
 *      Author: solomon
 */

#ifndef MOBILE_BASE_CONTROLLER_H_
#define MOBILE_BASE_CONTROLLER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include <cmath>

#include "trajectory_planner.h"

namespace mobile_base_controller {

class MobileBaseController: public trajectory_planner::TrajectoryPlanner {
public:
	MobileBaseController();
	virtual ~MobileBaseController();

	void setDynamicConstraints(float alpha_max, float omega_max, float a_max,
			float v_max);

	static float modulateAngle(float angle);

	bool planSmoothTrajectoryProfileRotation(float yaw_init, float yaw_goal,
			std::vector<float> &vct_omega, std::vector<float> &vct_phi);
	bool planSmoothTrajectoryProfileRectilinearTranslation(float s_init,
			float s_goal, std::vector<float> & vct_v,
			std::vector<float> & vct_s);

	bool planSmoothTrajectoryProfileRectilinearTranslationThroughMultipleWayPoints(
			std::vector<float> waypoints, std::vector<float> & vct_s,
			std::vector<float> & vct_v, std::vector<float> & vct_a);

	float getMaxLinearSpeed() {
		return m_v_max;
	}

	float getMaxAngularSpeed() {
		return m_omega_max;
	}

	float getMaxLinearAcc() {
		return m_a_max;
	}

	float getMaxAngularAcc() {
		return m_alpha_max;
	}

public:
	bool m_flag_dynamic_set;
	float m_a_max;
	float m_v_max;
	float m_alpha_max;
	float m_omega_max;

};

}
#endif /* MOBILE_BASE_CONTROLLER_H_ */
