#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/Vec4.h>
#include <nav_msgs/Odometry.h>
#include <mrs_lib/param_loader.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

namespace kfupm_test
{

class KfupmTest : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    ros::NodeHandle nh = getPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh, "KfupmTest");
    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam("odom_main_topic", odom_main_topic);
    param_loader.loadParam("control_service", control_service);
    // Subscriber
    sub_ = nh.subscribe(odom_main_topic, 1, &KfupmTest::odometryCallback, this);

    // Publisher
    pub_ = nh.advertise<std_msgs::String>("output_topic", 1);

    srv_client_control = nh.serviceClient<mrs_msgs::Vec4>(control_service);
    
    // Timer
    timer_ = nh.createTimer(ros::Duration(3.0), &KfupmTest::timerCallback, this, false);

    NODELET_INFO("KfupmTest initialized!");
  }

private:
  void odometryCallback(const nav_msgs::Odometry& msg)
  {
    if (!init_odom){
      initial_odom.x = msg.pose.pose.position.x;
      initial_odom.y = msg.pose.pose.position.y;

      wp1.x = initial_odom.x + 3;
      wp1.y = initial_odom.y;
      
      wp2.x = initial_odom.x - 3;
      wp2.y = initial_odom.y;

      init_odom = true;
    }

    current_odom.x = msg.pose.pose.position.x;
    current_odom.y = msg.pose.pose.position.y;
    ROS_INFO_THROTTLE(0.5, "UAV current: x: %f, y: %f, init: %f, %f", current_odom.x, current_odom.y, initial_odom.x, initial_odom.y);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    ROS_INFO("[KfupmTest]: Timer is running");
   
    if(init_odom){
      ROS_INFO("[KfupmTest]: 1");
      if(!wp1_done){
        ROS_INFO("[KfupmTest]: 2");
        mrs_msgs::Vec4 position_reference;
        position_reference.request.goal = {wp1.x, wp1.y, 2, 0.0};
        srv_client_control.call(position_reference);

        if(sqrt(pow(current_odom.x - wp1.x, 2) + pow(current_odom.y - wp1.y,2)) < 1.0){
          ROS_INFO("[KfupmTest]: 3");
          wp1_done = true;
        }
      } else{

        if(!wp2_done){
          ROS_INFO("[KfupmTest]: 4");
          mrs_msgs::Vec4 position_reference;
          position_reference.request.goal = {wp2.x, wp2.y, 2, 0.0};
          srv_client_control.call(position_reference);

          if(sqrt(pow(current_odom.x - wp2.x, 2) + pow(current_odom.y - wp2.y,2)) < 1.0){
            ROS_INFO("[KfupmTest]: 5");
            wp2_done = true;
          }
        } 

      }
    }

  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;

  bool init_odom = false;
  geometry_msgs::Point initial_odom;
  
  geometry_msgs::Point wp1;
  bool wp1_done = false;
  geometry_msgs::Point wp2;
  bool wp2_done = false;
  
  geometry_msgs::Point current_odom;

  ros::ServiceClient srv_client_control;
  
  std::string uav_name;
  std::string odom_main_topic;
  std::string control_service;
};

} // namespace kfupm_test

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kfupm_test::KfupmTest, nodelet::Nodelet)
