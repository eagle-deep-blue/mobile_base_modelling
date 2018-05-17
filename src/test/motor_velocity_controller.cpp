/*
 * motor_velocity_controller.cpp
 *
 *  Created on: Dec 25, 2017
 *      Author: solomon
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "motor_velocity_controller_node");
	ros::NodeHandle handle;

	ros::Publisher pub = handle.advertise<sensor_msgs::JointState>(
			"/joint_cmd_vel", 10);

	int size = argc - 1;

	if (size == 0) {
		ROS_ERROR("please set the motor velocity commands as argument!");
		return -1;
	} else {
		ROS_INFO_STREAM("[" << size << "] commands input!");
	}

	int rate = 6;
	while (ros::ok()) {
		sensor_msgs::JointState msg;
		msg.velocity.resize(size);
		for (int k = 0; k < size; k++) {
			msg.velocity.at(k) = std::atof(argv[k + 1]);
		}

		pub.publish(msg);

		ros::Rate(rate).sleep();
		ros::spinOnce();
	}

	return 0;
}

