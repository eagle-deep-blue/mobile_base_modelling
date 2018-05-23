/*
 * This ROS package models the motion for the mecanum omni-mobile base.
 *
 * It can receive mobile base command velocity in the form of geometry_msgs/Twist,
 *   and send out wheel command velocity in the form of sensor_msgs/JointState.
 *
 * Meanwhile, it can also compute the pose of the mobile base using the joint odometry, i.e.,
 *   the angular displacement of the wheels.
 *
 * ----  Developer: Solomon Jingchun YIN, jingchun.yin@live.com
 * ----  Date:    Nov-25th, 2017 
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "mecanum_omni_base.h"

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

using namespace mecanum_mobile_base;
boost::shared_ptr<MecanumOmniBase> mecanum_base;

ros::Publisher pub_joint_cmd;
ros::Publisher pub_joint_displacement;
ros::Publisher pub_odometry;
ros::Publisher pub_base_cmd_modulated;

bool flag_pub_tf;

void callbackBaseCmdVel ( const geometry_msgs::TwistConstPtr & topic );
void callbackWheelEncoder ( const sensor_msgs::JointStateConstPtr & msg );

void callbackTimer ( const ros::TimerEvent& );
void publishOdometry ( Eigen::Vector2f location, float theta,
                       Eigen::Vector3f speed_odometry );

const float transmission_ratio = 18.0;

int main ( int argc, char** argv )
{

  float width_left2right = 0.443;
  float length_front2back = 0.340;
  float wheel_radius = 0.0635;
  float angle_on_wheel = 45.0 / 180.0 * M_PI;

  ros::init ( argc, argv, "kinematics_mecnum_node" );
  ros::NodeHandle handle;

  float motor_max_speed;
  if ( !ros::param::get ( "~motor_max_speed", motor_max_speed ) )
    {
      motor_max_speed = 2 * M_PI;
    }
  float motor_min_speed;
  if ( !ros::param::get ( "~motor_min_speed", motor_min_speed ) )
    {
      motor_min_speed = 0.8 * M_PI;
    }

  mecanum_base = boost::shared_ptr<MecanumOmniBase> (
                   new MecanumOmniBase ( wheel_radius, angle_on_wheel, width_left2right,
                                         length_front2back, motor_max_speed, motor_min_speed,
                                         transmission_ratio, true ) );

  if ( !ros::param::get ( "~pub_tf", flag_pub_tf ) )
    {
      flag_pub_tf = true;
    }

  ROS_INFO_STREAM ( "information about the motion model of the mobile base:" );
  ROS_INFO_STREAM ( "motor_max_speed: -> [" << motor_max_speed << "]" );
  ROS_INFO_STREAM ( "motor_min_speed: -> [" << motor_min_speed << "]" );
  ROS_INFO_STREAM ( "pub tf -> [" << flag_pub_tf << "]" );

  std::string topic_cmd_vel;
  ros::param::param<std::string> ( "~topic_cmd_vel", topic_cmd_vel, "cmd_vel" );
  ros::Subscriber sub = handle.subscribe ( topic_cmd_vel, 10,
                        callbackBaseCmdVel );
  std::string topic_joint;
  ros::param::param<std::string> ( "~topic_joint_cmd_vel", topic_joint,
                                   "joint_cmd_vel" );
  pub_joint_cmd = handle.advertise<sensor_msgs::JointState> ( topic_joint, 10 );
  pub_base_cmd_modulated = handle.advertise<geometry_msgs::Twist> (
                             "cmd_vel_modulated", 10 );

  std::string topic_encoder;
  ros::param::param<std::string> ( "~topic_encoder", topic_encoder,
                                   "fake_joint_disp" );
  ros::Subscriber sub_joint_encoder = handle.subscribe ( topic_encoder, 10,
                                      callbackWheelEncoder );

  ros::Timer timer = handle.createTimer ( ros::Duration ( 0.1 ), callbackTimer );
  std::string topic_odometry;
  ros::param::param<std::string> ( "~topic_odometry", topic_odometry, "/odom" );
  pub_odometry = handle.advertise<nav_msgs::Odometry> ( topic_odometry, 10 );

  ros::spin();
  return 0;
}

void callbackBaseCmdVel ( const geometry_msgs::TwistConstPtr & topic )
{

  ROS_INFO ( "cmd_vel retrieved: [%f, %f, %f]!", topic->linear.x,
             topic->linear.y, topic->angular.z );


  sensor_msgs::JointStatePtr ptr_joint_cmd_vel ( new sensor_msgs::JointState() );
  ptr_joint_cmd_vel->header.stamp = ros::Time::now();
  ptr_joint_cmd_vel->name.resize ( 4 );
  ptr_joint_cmd_vel->name[0] = "front_left";
  ptr_joint_cmd_vel->name[1] = "front_right";
  ptr_joint_cmd_vel->name[2] = "back_right";
  ptr_joint_cmd_vel->name[3] = "back_left";
  ptr_joint_cmd_vel->velocity.resize ( 4 );

  Eigen::Vector3f base_cmd_vel ( topic->linear.x, topic->linear.y,
                                 topic->angular.z );
  Eigen::VectorXf joint_cmd_vel;
  Eigen::Vector3f base_cmd_vel_modulated;
  mecanum_base->transformBaseCmdVel2JointCmdVel ( base_cmd_vel, joint_cmd_vel,
      base_cmd_vel_modulated );

  float bound_linear = 0.3;
  if ( std::fabs ( base_cmd_vel_modulated ( 0 ) ) > bound_linear
       || std::fabs ( base_cmd_vel_modulated ( 1 ) ) > bound_linear )
    {
      ROS_ERROR_STREAM (
        "crazy modulate command -> [" << base_cmd_vel_modulated ( 0 ) << "," << base_cmd_vel_modulated ( 1 ) << "]" );

      ROS_INFO_STREAM (
        "related joint speed [" << joint_cmd_vel ( 0 ) << "," << joint_cmd_vel ( 1 ) << "," << joint_cmd_vel ( 2 ) << "," << joint_cmd_vel ( 3 ) << "]" );
      ROS_INFO_STREAM (
        "input base command -> [" << base_cmd_vel ( 0 ) << "," << base_cmd_vel ( 1 ) << "]" );
    }

  for ( int k = 0; k < 4; k++ )
    {
      ptr_joint_cmd_vel->velocity.at ( k ) = joint_cmd_vel ( k );
    }
  pub_joint_cmd.publish ( *ptr_joint_cmd_vel );

  geometry_msgs::Twist twist_modulated;
  twist_modulated.linear.x = base_cmd_vel_modulated ( 0 );
  twist_modulated.linear.y = base_cmd_vel_modulated ( 1 );
  twist_modulated.angular.z = base_cmd_vel_modulated ( 2 );
  pub_base_cmd_modulated.publish ( twist_modulated );


  ROS_INFO ( "joint_cmd_vel published: [%f, %f, %f, %f]",
             ptr_joint_cmd_vel->velocity[0], ptr_joint_cmd_vel->velocity[1],
             ptr_joint_cmd_vel->velocity[2], ptr_joint_cmd_vel->velocity[3] );


  return;
}

void callbackWheelEncoder ( const sensor_msgs::JointStateConstPtr & msg )
{

  //ROS_INFO("wheel encoder retrieved!");

  static bool flag_recorded = false;
  static Eigen::Vector4f joint_encoder_ref = Eigen::Vector4f::Zero();
  static double time_ref = ros::Time::now().toSec();

  Eigen::Vector4f joint_encoder = Eigen::Vector4f::Zero();
  for ( int k = 0; k < 4; k++ )
    {
      joint_encoder ( k ) = msg->position[k];
    }

  if ( flag_recorded )
    {
      Eigen::Vector4f dlt_encoder = joint_encoder - joint_encoder_ref;
      double time_dlt = ros::Time::now().toSec() - time_ref;


      ROS_INFO_STREAM (
        "dlt_encoder -> [" << dlt_encoder ( 0 ) << "," << dlt_encoder ( 1 ) << "," << dlt_encoder ( 2 ) << "," << dlt_encoder ( 3 ) <<"]" );

      Eigen::Vector3f speed_odom;
      mecanum_base->updateCurrentPoseUsingWheelEncoderIncrementalDisplacement (
        dlt_encoder, time_dlt );

    }

  joint_encoder_ref = joint_encoder;
  time_ref = ros::Time::now().toSec();
  flag_recorded = true;

}

void callbackTimer ( const ros::TimerEvent& )
{
  publishOdometry ( mecanum_base->getPosition(),
                    mecanum_base->getRotationMatrix().angle(),
                    mecanum_base->getSpeedOdometry() );

  ROS_INFO_STREAM ( "odometry published: [" << mecanum_base->getPosition() ( 0 ) << "," << mecanum_base->getPosition() ( 1 ) << "," << mecanum_base->getRotationMatrix().angle() <<"]" );

  std::cout << "\n\n";

}

void publishOdometry ( Eigen::Vector2f location, float theta,
                       Eigen::Vector3f speed_odometry )
{

  if ( flag_pub_tf )
    {
      static tf::TransformBroadcaster broadcaster;
      tf::Quaternion q = tf::createQuaternionFromYaw ( theta );
      tf::Vector3 t ( location ( 0 ), location ( 1 ), 0 );
      tf::Transform trans;
      trans.setOrigin ( t );
      trans.setRotation ( q );
      broadcaster.sendTransform (
        tf::StampedTransform ( trans, ros::Time::now(), "/odom",
                               "/base_link" ) );
    }

  nav_msgs::Odometry odom;
  odom.header.frame_id = "/odom";
  odom.header.stamp = ros::Time::now();
  odom.child_frame_id = "/base_link";
  odom.pose.pose.position.x = location ( 0 );
  odom.pose.pose.position.y = location ( 1 );
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw ( theta );

  odom.twist.twist.linear.x = speed_odometry ( 0 );
  odom.twist.twist.linear.y = speed_odometry ( 1 );
  odom.twist.twist.angular.z = speed_odometry ( 2 );

  pub_odometry.publish ( odom );

}

