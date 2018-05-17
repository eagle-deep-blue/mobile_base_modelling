/*
 * test_cmd_vel.cpp
 *
 *  Created on: Dec 22, 2017
 *      Author: solomon
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <cstdlib>


int main(int argc, char** argv) {

	ros::init(argc, argv, "test_cmd_vel_node");
	ros::NodeHandle handle;

	ros::Publisher pub = handle.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	double vx = std::atof(argv[1]);
	double vy = std::atof(argv[2]);
	double omega = std::atof(argv[3]);

	while (ros::ok()) {
		geometry_msgs::Twist cmd;
		cmd.linear.x = vx;
		cmd.linear.y = vy;
		cmd.angular.z = omega;

		pub.publish(cmd);

		ros::Rate(10).sleep();
		ros::spinOnce();
	}

	ros::spin();

	return 0;
}
