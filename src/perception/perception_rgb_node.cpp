/*
 * perception_ros_node.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: solomon
 */

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>

void
callbackImage (const sensor_msgs::ImageConstPtr & img_ptr);

int cnt;
int idx;

int
main (int argc, char** argv)
{

  cnt = 0;
  idx = 0;

  ros::init (argc, argv, "image_retriever");

  ros::NodeHandle handler;
  image_transport::ImageTransport it (handler);

  std::string topic_img = "/usb_cam/image_raw";
  image_transport::Subscriber it_sub_img = it.subscribe (topic_img, 100,
							 callbackImage);

  ros::spin ();

  return 0;
}

void
callbackImage (const sensor_msgs::ImageConstPtr & msg_img)
{

  cv_bridge::CvImagePtr img_ptr;
  try
    {
      if (sensor_msgs::image_encodings::isColor (msg_img->encoding))
	{
	  img_ptr = cv_bridge::toCvCopy (msg_img,
					 sensor_msgs::image_encodings::BGR8);
	}
      else
	{
	  img_ptr = cv_bridge::toCvCopy (msg_img,
					 sensor_msgs::image_encodings::MONO8);

	}

      if (img_ptr->image.data)
	{

	  cv::Mat image = img_ptr->image;

	  cv::imshow ("raw", image);
	  cv::waitKey (3);
  //   if(cnt% 2 == 0){
      char fn[30];
      sprintf(fn, "rawoutput%04d.pgm", idx);

      std::vector<int> param;
      param.push_back(CV_IMWRITE_PXM_BINARY);
      param.push_back(1);

      cv::imwrite(fn, image, param);

      idx++;
   //  }

	  ROS_INFO("image data retireved!\n");
	}

    }
  catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("error in image data retrieval!\n");
    }

  cnt++;
}

