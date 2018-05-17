/*
 * unicycle_base_controller.h
 *
 *  Created on: Jan 29, 2018
 *      Author: solomon
 */

#ifndef unicycle_base_controller_H_
#define unicycle_base_controller_H_

#include "mobile_base_controller.h"
#include <Eigen/Core>
#include <vector>
#include <iostream>

namespace unicycle_base_controller {

class UnicycleBaseController {
public:
	UnicycleBaseController();
	virtual ~UnicycleBaseController();

	// given the state of the unicycle robot, and the offset of the line path to track,
	// compute the angular velocity;
	void trackRectilinearPath(Eigen::Vector2f point_on_line,
			Eigen::Vector2f tangent_vct, Eigen::Vector2f position, float theta,
			float dist_threshold, float speed_constant, float K_phi,
			float omega_max, float & omega);

	std::vector<Eigen::Vector2f> selectSignificantPointsFromGeometricWaypointSet(
			std::vector<Eigen::Vector2f> vct_waypoints);

};

} /* namespace unicycle_base_controller */

#endif /* unicycle_base_controller_H_ */
