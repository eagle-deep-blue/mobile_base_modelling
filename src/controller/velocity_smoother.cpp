/*
 * velocity_smoother.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: solomon
 */

/*
 * this package takes in a base velocity command,
 * and sends out a sequence of base velocity commands in a smooth way;
 *
 * if no base velocity input, then only zero base velocity command is sent out;
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <cmath>

geometry_msgs::Twist twist_desired;
geometry_msgs::Twist twist;

float increment_linear;
float increment_angular;

ros::Publisher pub;
bool flag_desired_cmd_taken;

void computeSmoothCmdVel(double desired, double &actual, double increment);
void callbackSendContorlCmdVel(const ros::TimerEvent & timer);
void callbackDesiredCmdVel(const geometry_msgs::TwistConstPtr & msg);

int main(int argc, char** argv) {

	ros::init(argc, argv, "velocity_smoother");
	ros::NodeHandle handle;

	twist_desired.linear.x = 0.0;
	twist_desired.linear.y = 0.0;
	twist_desired.angular.z = 0.0;

	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.angular.z = 0.0;

	flag_desired_cmd_taken = false;

	if (!handle.getParam("value_increment_linear", increment_linear)) {
		increment_linear = 0.05;
	}

	if (!handle.getParam("value_increment_angular", increment_angular)) {
		increment_angular = 5.0 / 180.0 * M_PI;
	}

	ros::Subscriber sub = handle.subscribe("cmd_vel", 10,
			callbackDesiredCmdVel);

	pub = handle.advertise<geometry_msgs::Twist>("cmd_vel_smooth", 10);
	ros::Timer timer = handle.createTimer(ros::Duration(0.1),
			callbackSendContorlCmdVel);

	ros::spin();

	return 0;
}

void computeSmoothCmdVel(double desired, double &actual, double increment) {
	if (desired > actual) {
		actual = std::min(actual + increment, desired);
	} else if (desired < actual) {
		actual = std::max(actual - increment, desired);
	} else {
		actual = desired;
	}
}

void callbackSendContorlCmdVel(const ros::TimerEvent & timer) {

	if (flag_desired_cmd_taken) {
		computeSmoothCmdVel(twist_desired.linear.x, twist.linear.x,
				increment_linear);
		computeSmoothCmdVel(twist_desired.linear.y, twist.linear.y,
				increment_linear);

		computeSmoothCmdVel(twist_desired.angular.z, twist.angular.z,
				increment_angular);
	} else {
		computeSmoothCmdVel(0.0, twist.linear.x, increment_linear);
		computeSmoothCmdVel(0.0, twist.linear.y, increment_linear);

		computeSmoothCmdVel(0.0, twist.angular.z, increment_angular);
	}

	pub.publish(twist);
	ROS_INFO_STREAM("smooth velocity command output:[" << twist.linear.x << "," << twist.linear.y << "," << twist.angular.z << "]");
	flag_desired_cmd_taken = false;

	return;
}

void callbackDesiredCmdVel(const geometry_msgs::TwistConstPtr & msg) {

	ROS_INFO("velocity command got!");

	twist_desired.linear.x = msg->linear.x;
	twist_desired.linear.y = msg->linear.y;
	twist_desired.angular.z = msg->angular.z;

	flag_desired_cmd_taken = true;

	return;
}

