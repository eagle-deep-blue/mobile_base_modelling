/*
 * unicycle_controller.cpp
 *
 *  Created on: Jan 29, 2018
 *      Author: solomon
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <ros/timer.h>

#include "unicycle_base_controller.h"
#include <boost/shared_ptr.hpp>

boost::shared_ptr<unicycle_base_controller::UnicycleBaseController> unicycle_controller;
ros::Publisher pub_cmd_vel;
float speed_constant = 0.2;
float x_latest;
float y_latest;
float theta_latest;
float Kphi;

void callbackOdom(const nav_msgs::OdometryConstPtr & msg);
void callbackTimer(const ros::TimerEvent & msg);

int main(int argc, char** argv) {
	unicycle_controller = boost::shared_ptr<
			unicycle_base_controller::UnicycleBaseController>(
			new unicycle_base_controller::UnicycleBaseController);

	ros::init(argc, argv, "unicycle_controller_node");
	ros::NodeHandle handle;

	if (!ros::param::get("coefficient_phi_err", Kphi)) {
		Kphi = 1.2;
	}

	ros::Subscriber sub_odom = handle.subscribe("odom", 10, callbackOdom);

	ros::Timer timer = handle.createTimer(ros::Duration(0.01), callbackTimer);
	pub_cmd_vel = handle.advertise<geometry_msgs::Twist>(
			"mobile_base/commands/velocity", 10);

	ros::spin();
	return 0;
}

void callbackOdom(const nav_msgs::OdometryConstPtr & msg) {

	if (!msg)
		ROS_ERROR("NO odometry data!!!");

	x_latest = msg->pose.pose.position.x;
	y_latest = msg->pose.pose.position.y;
	theta_latest = tf::getYaw(msg->pose.pose.orientation);

	return;
}

void callbackTimer(const ros::TimerEvent & msg) {
	float omega;
	Eigen::Vector2f point_on_line(0.0, 0.0);
	Eigen::Vector2f tangent_vct(1.0, 0.0);
	unicycle_controller->trackRectilinearPath(point_on_line, tangent_vct,
			Eigen::Vector2f(x_latest, y_latest), theta_latest, 0.5,
			speed_constant, Kphi, 10.0 / 180.0 * M_PI, omega);

	geometry_msgs::Twist twist;
	twist.linear.x = speed_constant;
	twist.angular.z = omega;
	pub_cmd_vel.publish(twist);

	ROS_INFO_STREAM(
			"current pose [" << x_latest << "," << y_latest << "," << theta_latest << "] with cmd_vel [" << speed_constant << "," << omega<<"]");

	return;
}

