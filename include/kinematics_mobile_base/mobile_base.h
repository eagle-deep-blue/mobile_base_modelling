/*
 * This package is the base class for the mobile base;
 *
 * It abstracts the mobile base as a complete unit without considering the details of kinematics;
 *
 * It defines the virtual interface to transform the command velocity from cartesian space of the mobile base
 * 		to the joint space of the wheel and vice versa.
 * And these virtual functions are to be defined by the derived classes according to the kinematics features
 * 		of the particular mobile base;
 *
 * It can update the pose of the mobile base using the incremental value in local cartesian frame.
 * For the way to update the pose of the mobile base using the kinematical features of particular mobile platform,
 * there is virtual function interface.
 *
 * ----  Developer: Solomon Jingchun YIN, jingchun.yin@live.com
 * ----  Date:    Nov-25th, 2017 
 *
 */

#ifndef MOBILEBASE_H
#define MOBILEBASE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

class MobileBase {
public:
	MobileBase();
	~MobileBase();

	virtual bool transformBaseCmdVel2JointCmdVel(Eigen::Vector3f base_cmd_vel,
			Eigen::VectorXf &joint_cmd_vel,
			Eigen::Vector3f &base_cmd_vel_modulated) = 0;

	virtual bool modulateJointCmdVel(Eigen::Vector3f base_cmd_vel,
			Eigen::VectorXf joint_cmd_vel,
			Eigen::VectorXf & joint_cmd_vel_modulated,
			Eigen::Vector3f &base_cmd_vel_modulated) = 0;

	virtual bool transformJointCmdVel2BaseCmdVel(Eigen::VectorXf joint_cmd_vel,
			Eigen::Vector3f &base_cmd_vel) = 0;

	virtual bool transformJointIncrementalDisplacement2BaseIncrementalDisplacement(
			Eigen::Vector3f &base_cartesian_displacement,
			Eigen::VectorXf joint_angular_displacement) = 0;

	virtual bool updateCurrentPoseUsingWheelEncoderIncrementalDisplacement(
			Eigen::VectorXf joint_angular_displacement, double time_dlt) = 0;

	void updateCurrentPose(Eigen::Vector3f base_cartesian_displacement,
			double time_dlt);

	Eigen::Vector2f getPosition() {
		return m_position;
	}

	Eigen::Rotation2Df getRotationMatrix() {
		return *m_rot;
	}

	Eigen::Vector3f getSpeedOdometry() {
		return m_speed_odometry;
	}

private:
	Eigen::Vector2f m_position;
	Eigen::Rotation2Df* m_rot;

	Eigen::Vector3f m_speed_odometry;
};

#endif // MOBILEBASE_H
