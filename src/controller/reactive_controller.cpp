/*
 * This package reads in the message from the bumper, 
 * and sends out the reaction safety velocity command to avoid the emergency situation;
 * 
 * Author: Solomon Jingchun YIN, jingchun.yin@live.com, May 15th, 2018;
 *   
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Bool.h>

#include <std_msgs/UInt16MultiArray.h>

#include <Eigen/Core>


  ros::Publisher pub;
void callbackBumper(const std_msgs::UInt16MultiArray & msg);
void callbackOdometry(const nav_msgs::OdometryConstPtr & msg);

Eigen::Vector3f gOdom;

int main(int argc, char** argv){
  
    gOdom = Eigen::Vector3f::Zero();
  
  ros::init(argc, argv, "reactive_controller_node");
  ros::NodeHandle handle;  
  pub = handle.advertise<geometry_msgs::Twist>("cmd_vel",10);
  ros::Subscriber sub = handle.subscribe("robot/bumper", callbackBumper);
  ros::Subscriber sub = handle.subscribe("odom", callbackOdometry);
  
  ros::spin();
  
  return 0;
}


void callbackOdometry(const nav_msgs::OdometryConstPtr & msg){
 
  gOdom(0) = msg->pose.pose.position.x;
  gOdom(1) = msg->pose.pose.position.y;
  tf::Quaternion q_tf;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q_tf);  
  gOdom(2) =tf::getYaw(q_tf);
  
  return;
}


void callbackBumper ( const std_msgs::UInt16MultiArray& msg )
{
//   ROS_INFO_STREAM("bumper -> [" << msg.data[0] << "," << );
  
  bool flag_front, flag_back, flag_right, flag_left;
  
  if(msg.data[0] == 1)
    flag_front = true;
  else
    flag_front = false;
  
  if(msg.data[1] == 1)
    flag_back = true;
  else
    flag_back = false;
  
  if(msg.data[2]==1)
    flag_right = true;
  else 
    flag_right = false;
  
  if(msg.data[3] == 1)
    flag_left = true;
  else
    flag_left = false;
  
  geometry_msgs::Twist twist;
  
  float speed = 0.1;
  
  if(flag_front){
   ROS_INFO("FRONT bumper attacked!"); 
   twist.linear.x = -1 * speed;
  }
  else if(flag_back){
   ROS_INFO("BACK bumper attacked!"); 
   twist.linear.x = speed;
   
  }else if(flag_right){
   ROS_INFO("RIGHT bumper attacked!"); 
   
   
  }
  else if(flag_left){
   ROS_INFO("LEFT bumper attacked!"); 
  }
  
  
  
  
  
  
  
  
  
  
  return;
}

