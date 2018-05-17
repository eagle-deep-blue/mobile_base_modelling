/*
 * perception_laser_scan.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: solomon
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void callbackLaser(const sensor_msgs::LaserScanConstPtr& msg);

int main(int argc, char** argv) {

	ros::init(argc, argv, "perception_laser_scan_node");
	ros::NodeHandle handle;

	ros::Subscriber sub = handle.subscribe("scan", 10, callbackLaser);

	ros::spin();

	return 0;
}

float radian2degree(float radian) {
	return radian / M_PI * 180;

}

float degree2radian(float degree) {
	return degree / 180.0 * M_PI;
}

void callbackLaser(const sensor_msgs::LaserScanConstPtr& msg) {

	ROS_INFO("scan retrieved!");

	float dlt = 30;
	float angle_bn1 = degree2radian(-90 - 30);
	float angle_bn2 = degree2radian(-90 + 30);

	for (int k = 0; k < msg->ranges.size(); k++) {
		float angle = msg->angle_min + k * msg->angle_increment;

		if (angle > angle_bn1 && angle < angle_bn2) {
			ROS_INFO_STREAM(
					"["<< k << "] -> [" << angle << "] -> [" << msg->ranges[k] << "]");
		}

	}

	/*
	 int idx_bn1 = -(msg->angle_min - angle_bn1) / msg->angle_increment;
	 int idx_bn2 = -(msg->angle_min - angle_bn2) / msg->angle_increment;
	 ROS_INFO_STREAM("index range[" << idx_bn1 << "," << idx_bn2 << "]");
	 for (int k = 0; k > idx_bn1 && k < idx_bn2; k++) {
	 ROS_INFO_STREAM("[" << k << "] -> [" << msg->ranges[k] << "]");
	 }*/

	/*
	 ROS_INFO_STREAM(
	 "laser stream received with [" << msg->ranges.size() << "] elements");

	 ROS_INFO_STREAM("laser frame -> [" << msg->header.frame_id << "]");

	 int size = (msg->angle_max - msg->angle_min) / msg->angle_increment;
	 std::cout << "angle gap size -> [" << size << "]\n";

	 ROS_INFO_STREAM(
	 "angle -> min [" << msg->angle_min/M_PI * 180 << "] v.s. max [" << msg->angle_max/M_PI * 180 << "] at res. [" << msg->angle_increment << "]");

	 float threshold_dist = 0.3;
	 for (int k = 0; k < msg->ranges.size(); k++) {
	 if (msg->ranges[k] < threshold_dist) {
	 ROS_INFO_STREAM(
	 "near object -> dist [" << msg->ranges[k] << "] at angle [" << radian2degree(msg->angle_min + msg->angle_increment * k )<< "]");
	 }
	 }*/

	return;
}

