/*
 * This ROS package models the fake wheel encoder odometry based on joint command velocity.
 *
 * ----  Developer: Solomon Jingchun YIN, jingchun.yin@live.com
 * ----  Date:    Nov-25th, 2017 @ partnerX Robotics Corp., Shanghai, P.R.China;
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

void callbackJointCmd(const sensor_msgs::JointStateConstPtr & msg);
ros::Publisher pub_joint_displacement;

int main(int argc, char** argv) {

	ros::init(argc, argv, "fake_wheel_encoder_node");
	ros::NodeHandle handle;

	ros::Subscriber sub_joint_cmd = handle.subscribe("joint_cmd_vel", 10,
			callbackJointCmd);
	pub_joint_displacement = handle.advertise<sensor_msgs::JointState>(
			"fake_joint_disp", 10);

	ros::spin();

	return 0;
}

void callbackJointCmd(const sensor_msgs::JointStateConstPtr & msg) {
	static Eigen::Vector4f joint_angular_displacement = Eigen::Vector4f::Zero();

	static bool flag_recorded = false;
	static Eigen::Vector4f joint_cmd_vel_ref = Eigen::Vector4f::Zero();
	static double time_ref_joint = ros::Time::now().toSec();

	Eigen::Vector4f joint_cmd_vel = Eigen::Vector4f::Zero();
	for (int k = 0; k < 4; k++) {
		joint_cmd_vel(k) = msg->velocity[k];

	}

	if (flag_recorded) {
		float dlt_time_wheel = ros::Time::now().toSec() - time_ref_joint;

		ROS_INFO_STREAM("retrieved! -> " << dlt_time_wheel);

		if (dlt_time_wheel < 1) {
			Eigen::Vector4f dlt_joint_angular_displacement = joint_cmd_vel_ref
					* dlt_time_wheel;
			joint_angular_displacement += dlt_joint_angular_displacement;

			sensor_msgs::JointStatePtr fake_joint_disp(
					new sensor_msgs::JointState());
			fake_joint_disp->header.stamp = ros::Time::now();
			fake_joint_disp->name.resize(4);
			fake_joint_disp->name[0] = "front_left";
			fake_joint_disp->name[1] = "front_right";
			fake_joint_disp->name[2] = "back_right";
			fake_joint_disp->name[3] = "back_left";
			fake_joint_disp->velocity.resize(4);
			for (int k = 0; k < 4; k++) {
				fake_joint_disp->velocity[k] = msg->velocity[k];

			}

			fake_joint_disp->position.resize(4);
			for (int k = 0; k < 4; k++) {
				fake_joint_disp->position[k] = joint_angular_displacement(k);
			}
			pub_joint_displacement.publish(fake_joint_disp);

			ROS_INFO_STREAM(
					"encoder [" << fake_joint_disp->position[0] << "," << fake_joint_disp->position[1] << "," << fake_joint_disp->position[2] << "," << fake_joint_disp->position[3]);
		}
	}

	joint_cmd_vel_ref = joint_cmd_vel;
	time_ref_joint = ros::Time::now().toSec();
	flag_recorded = true;

	return;
}
