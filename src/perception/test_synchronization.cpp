#include <ros/ros.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

void callbackSync(const sensor_msgs::ImageConstPtr &msg_img, const sensor_msgs::ImuConstPtr & msg_imu);


int main(int argc, char** argv){
  
  ros::init(argc, argv, "test_sync_ndoe");
  
  ros::NodeHandle handle;
  
  typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  sensor_msgs::Imu> SyncPolicy;
  
  std::string topic_imu = "/imu/data";
  std::string topic_img = "/camera/image_raw";
  
  ROS_INFO_STREAM("topic_img [" << topic_img << "]");
  ROS_INFO_STREAM("topic_imu [" << topic_imu << "]");
  
  message_filters::Subscriber<sensor_msgs::Imu> sub_imu(handle, topic_imu, 200);
  message_filters::Subscriber<sensor_msgs::Image> sub_img(handle, topic_img, 200);
  
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img, sub_imu);
  sync.registerCallback(boost::bind(&callbackSync, _1, _2));
  
  
//   while(ros::ok()){
//    
//     ROS_INFO("ok!");
//     
//   }
//   
 
  ros::spin();
 
  return 0;
}

void callbackSync(const sensor_msgs::ImageConstPtr &msg_img, const sensor_msgs::ImuConstPtr & msg_imu){
  
  
    ROS_INFO("sync called ");
    ROS_INFO_STREAM("imu -> ["<< msg_imu->header.stamp.toNSec() << "] v.s. img -> [" << msg_img->header.stamp.toNSec() << "]");
  
}









