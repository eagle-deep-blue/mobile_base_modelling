/*
 * rgbd_cup_detection_node.cpp
 *
 *  Created on: Sep 20, 2017
 *      Author: solomon
 */

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void
callbackIMUCOLOR (const sensor_msgs::ImuConstPtr& msg_imu,
		  const sensor_msgs::ImageConstPtr & msg_img);

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "rgbd_capturer_node");
  ros::NodeHandle handler;

  std::string topic_imu = "/imu/data";
  std::string topic_img = "/camera/rgb/image_raw";

  message_filters::Subscriber<sensor_msgs::Imu> sub_msg_flt_imu (handler,
								 topic_imu,
								 100);
  message_filters::Subscriber<sensor_msgs::Image> sub_msg_flt_rgb (handler,
								   topic_img,
								   100);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,
      sensor_msgs::Image> SycPlc;
  message_filters::Synchronizer<SycPlc> sync (SycPlc (10), sub_msg_flt_imu,
					      sub_msg_flt_rgb);

  sync.registerCallback (boost::bind (&callbackIMUCOLOR, _1, _2));

  ros::spin ();
  return 0;
}

void
callbackIMUCOLOR (const sensor_msgs::ImuConstPtr& msg_imu,
		  const sensor_msgs::ImageConstPtr & msg_img)
{

  ROS_INFO("callback invoked!");

  /*
   cv_bridge::CvImagePtr ptr_img;
   try
   {
   ptr_img = cv_bridge::toCvCopy (msg_img,
   sensor_msgs::image_encodings::BGR8);
   if (ptr_img->image.data)
   ROS_INFO("rgb retrieved!");
   }
   catch (cv_bridge::Exception &e)
   {
   ROS_ERROR("error in capturing depth images!");
   return;
   }

   if (ptr_img->image.data)
   {
   ROS_INFO("color image retrieved!");
   cv::imshow ("color", ptr_img->image);
   cv::waitKey (3);
   }
   */
}
