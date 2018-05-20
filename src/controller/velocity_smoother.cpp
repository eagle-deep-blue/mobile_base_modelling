
/*
 * This package takes in the velocity command [geometry_msgs/Twist],
 * then it computes and sends out a sequence of smooth velocity command [geometry_msgs/Twist];
 *
 * when in the very beginning, zero velocity is sent out;
 * if long-time elapsed since last input velocity command, then the velocity command sent out gradually decay to zero;
 * if stop flag is retrieved, then sharp zero velocity command is sent out;
 *
 * Author: Solomon Jingchun YIN, jingchu.yin@live.com, May 14th, 2018;
 *
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <ros/timer.h>

#include <Eigen/Core>
#include <iostream>

void callbackTwist ( const geometry_msgs::TwistConstPtr & msg );
void callbackStop ( const std_msgs::BoolConstPtr & msg );
void callbackTimer ( const ros::TimerEvent &event );

Eigen::Vector3f target_speed;
double target_timestamp;
Eigen::Vector3f control_speed;

float limit_linear;
float limit_angular;
float increments_linear;
float increments_angular;
float receive_cmd_timeout;
bool holonomic_robot;

std::string cmd_vel_input;
std::string cmd_vel_output;

ros::Publisher pub;
// ros::Publisher pub_cmd;
bool flag_stop;

bool setTargetValue ( float value, float limit, float& value_modulates );
void setControlSpeed ( float &control_speed, float increment, float target_speed );

int main ( int argc, char** argv )
{

  flag_stop = false;

  control_speed = Eigen::Vector3f::Zero();
  target_speed = Eigen::Vector3f::Zero();

  ros::init ( argc, argv,"cmd_vel_smoother" );
  ros::NodeHandle handle;
  ros::Subscriber sub_flag_stop = handle.subscribe ( "flag_stop",10, callbackStop );

  if ( !ros::param::get( "~limit_linear", limit_linear ) )
    limit_linear = 0.6;

  if ( !ros::param::get ( "~limit_angular", limit_angular ) )
    limit_angular= 0.5;

  if ( !ros::param::get( "~increments_linear", increments_linear ) )
    increments_linear= 0.02;

if ( !ros::param::get( "~increments_angular", increments_angular ) )
    increments_angular= 0.06;

  if ( !ros::param::get( "~receive_cmd_timeout", receive_cmd_timeout ) )
    receive_cmd_timeout = 0.6;

  if ( !ros::param::get( "~holonomic_robot", holonomic_robot ) )
    holonomic_robot = true;
  
   if(!ros::param::get("~cmd_vel_input", cmd_vel_input))
     cmd_vel_input = "cmd_vel";  
  if(!ros::param::get("~cmd_vel_output", cmd_vel_output))
    cmd_vel_output = "cmd_vel_smooth";
  
  ROS_INFO_STREAM ("-----------------------------------------");
  ROS_INFO_STREAM ( "limit linear [" << limit_linear << "]" );
  ROS_INFO_STREAM ( "limit angular [" << limit_angular << "]" );
  ROS_INFO_STREAM ( "increments_linear [" << increments_linear << "]" );
  ROS_INFO_STREAM ( "increments_angular [" << increments_angular << "]" );
  ROS_INFO_STREAM ( "receive_cmd_timeout [" << receive_cmd_timeout<< "]" );
  ROS_INFO_STREAM ( "holonomic_robot [" << holonomic_robot << "]" );
  ROS_INFO_STREAM("input command [" << cmd_vel_input << "]");
  ROS_INFO_STREAM("output command [" << cmd_vel_output << "]");
  ROS_INFO_STREAM ("-----------------------------------------");  

    ros::Subscriber sub = handle.subscribe ( cmd_vel_input, 10,callbackTwist ); 

  
  pub = handle.advertise<geometry_msgs::Twist> ( cmd_vel_output,10 );
  ros::Timer timer = handle.createTimer ( ros::Duration ( 0.1 ),callbackTimer );
  target_timestamp = 0.0;

  ros::spin();
  return 0;
}

bool setTargetValue ( float value, float limit, float &value_modulated )
{
  if ( limit == 0 )
    return false;

  if ( std::fabs ( value ) > std::abs ( limit ) )
    {
      value_modulated= limit * value / std::fabs ( value );
    }
  else
    {
      value_modulated = value;
    }


  return true;
}

void callbackStop ( const std_msgs::BoolConstPtr & msg )
{

  if ( msg->data )
    {
      flag_stop = true;
      target_speed = Eigen::Vector3f::Zero();
      target_timestamp = ros::Time::now().toSec();
    }
  else
    flag_stop = false;

  return ;
}

void callbackTwist ( const geometry_msgs::TwistConstPtr & msg )
{
  float x = msg->linear.x;
  float y;

  if ( holonomic_robot )
    y= msg->linear.y;
  else
    y = 0.0;

  float theta = msg->angular.z;

  float x_modulated = 0.0;
  float y_modulated = 0.0;
  float theta_modulated = 0.0;

  if ( !setTargetValue ( x, limit_linear,x_modulated ) || !setTargetValue ( y, limit_linear, y_modulated )
       || !setTargetValue ( theta, limit_angular, theta_modulated ) )
    {
      return;
    }

  target_speed = Eigen::Vector3f ( x_modulated, y_modulated, theta_modulated );
  target_timestamp = ros::Time::now().toSec();

  return ;
}

void setControlSpeed ( float &control_speed, float increment, float target_speed )
{

  if ( control_speed< target_speed )
    {
      control_speed  = std::min ( control_speed  + increment, target_speed ) ;
    }
  else if ( control_speed > target_speed )
    {
      control_speed  = std::max ( control_speed - increment, target_speed );
    }
  else if ( control_speed  == target_speed )
    {
      control_speed= target_speed;
    }

  return ;

}


void callbackTimer ( const ros::TimerEvent & msg )
{
  // in case of emergency, sharp stop;
  if ( flag_stop )
    {
      control_speed = Eigen::Vector3f::Zero();
      
//       ROS_INFO_STREAM("control speed [" << control_speed.transpose() << "]");
      
      geometry_msgs::Twist twist;
      int cnt = 0;
      while ( cnt < 6 )
        {
          pub.publish ( twist );
          ros::Duration ( 0.1 ).sleep();
          cnt++;
        }

      flag_stop = false;
      return;
    }

  // reset the target command velocity if time is out;
  double time_since_last_target = ros::Time::now().toSec() - target_timestamp;
  if ( time_since_last_target > receive_cmd_timeout )
    target_speed = Eigen::Vector3f::Zero();
//   ROS_INFO_STREAM ( "target speed [" << target_speed.transpose() << "]" );

  // compute the smooth control velocity command;
  setControlSpeed ( control_speed ( 0 ),increments_linear, target_speed ( 0 ) );
  if ( holonomic_robot )
    setControlSpeed ( control_speed ( 1 ), increments_linear, target_speed ( 1 ) );
  else
    control_speed ( 1 ) = 0.0;
  setControlSpeed ( control_speed ( 2 ), increments_angular, target_speed ( 2 ) );

  // in case of too many zero control speed, escape;
  static int cnt_zeros = 0;
  Eigen::Vector3f aux = control_speed - Eigen::Vector3f::Zero();
  float eps = 0.01;
  if ( aux.norm() < eps )
    {
      cnt_zeros++;
    }
  else
    {
      cnt_zeros = 0;
    }

  if ( cnt_zeros > 5 )
    {
//       ROS_INFO_STREAM ( "stop publishing any twist message!" );
      return;
    }

  geometry_msgs::Twist twist_output;
  twist_output.linear.x = control_speed ( 0 );
  twist_output.linear.y = control_speed ( 1 );
  twist_output.angular.z = control_speed ( 2 );
  pub.publish ( twist_output );
//   ROS_INFO_STREAM ( "publishing the twist message!" );
//   ROS_INFO_STREAM("control speed [" << control_speed.transpose() << "]");

}

