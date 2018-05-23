/*
 * This package implements the kinematic motion algorithm for the mecanum omni-directional base.
 *
 * It has interface to transform the command velocity from the cartesian space of the mobile base
 * to the joint space of the wheels, and vice versa.
 *
 * It can also update the pose the mobile base using the incremental angular displacement from the wheel encoders.
 *
 * Note that the command velocity of the wheel should be amplified by the transmission ratio
 *      before it is sent to the motor for actuation.
 *
 * ----  Developer: Solomon Jingchun YIN, jingchun.yin@live.com
 * ----  Date:    Nov-25th, 2017 
 *
 */

#ifndef MECANUMOMNIBASE_H
#define MECANUMOMNIBASE_H

#include "mobile_base.h"

namespace mecanum_mobile_base {

class MecanumOmniBase: public MobileBase {
public:
	// note that the default values are already specified for the mecanum-omni mobile base;
	MecanumOmniBase(float wheel_radius = 0.0635,
			float angle_on_wheel = 45.0 / 180.0 * M_PI, float width_left2right =
					0.443, float length_front2back = 0.340,
			float speed_limit_motor = 2 * 2 * M_PI,
			float speed_limit_motor_min = 1.6 * 2 * M_PI,
			float transmission_ratio = 18.0, bool flag_debug = false);

	~MecanumOmniBase();

	// set the geometric parameters;
	void setParameters(float wheel_radius, float angle_on_wheel,
			float width_left2right, float length_front2back,
			float speed_limit_motor, float speed_limit_motor_min,
			float transmission_ratio);

	// base command velocity [vx, vy, omega]
	//      -  with x direction pointing forwards, and y direction to the left;
	// 		-  {vx, vy} -> unit of [m/sec],
	//		-  {omega} -> unit of [rad/sec]
	// joint command velocity is in unit of [rad/sec],
	//		with each wheel rotating to move the base forward as positive direction:	

	bool transformBaseCmdVel2JointCmdVel(Eigen::Vector3f base_cmd_vel,
			Eigen::VectorXf &joint_cmd_vel,
			Eigen::Vector3f &base_cmd_vel_modulated);

	bool modulateJointCmdVel(Eigen::Vector3f base_cmd_vel,
			Eigen::VectorXf joint_cmd_vel,
			Eigen::VectorXf & joint_cmd_vel_modulated,
			Eigen::Vector3f &base_cmd_vel_modulated);

	bool transformJointCmdVel2BaseCmdVel(Eigen::VectorXf joint_cmd_vel,
			Eigen::Vector3f &base_cmd_vel);

	bool transformJointIncrementalDisplacement2BaseIncrementalDisplacement(
			Eigen::Vector3f &base_cartesian_displacement,
			Eigen::VectorXf joint_angular_displacement);

	bool updateCurrentPoseUsingWheelEncoderIncrementalDisplacement(
			Eigen::VectorXf dlt_joint_angular_displacement, double time_dlt);

private:
	bool m_flag_debug;
	float m_speed_max_joint;
	float m_speed_min_joint;
	float m_wheel_radius;
	float m_angle_on_wheel;
	float m_width_left2right;
	float m_length_front2back;

	float m_transmission_ratio;
};

}

#endif // MECANUMOMNIBASE_H
