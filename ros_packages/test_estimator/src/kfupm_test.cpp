#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/Vec4.h>
#include <nav_msgs/Odometry.h>
#include <mrs_lib/param_loader.h>
#include <std_msgs/String.h>

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
    timer_ = nh.createTimer(ros::Duration(5.0), &KfupmTest::timerCallback, this, false);

    NODELET_INFO("KfupmTest initialized!");
  }

private:
  void odometryCallback(const nav_msgs::Odometry& msg)
  {
    ROS_INFO_THROTTLE(0.5, "UAV is at coordinates: x: %f, y: %f", msg.pose.pose.position.x, msg.pose.pose.position.y);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    ROS_INFO("[KfupmTest]: Timer is running");
    mrs_msgs::Vec4 position_reference;
    position_reference.request.goal = {2.0, 0.0, 0.0, 0.0};
    srv_client_control.call(position_reference);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
    
  ros::ServiceClient srv_client_control;
  
  std::string uav_name;
  std::string odom_main_topic;
  std::string control_service;
};

} // namespace kfupm_test

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kfupm_test::KfupmTest, nodelet::Nodelet)
