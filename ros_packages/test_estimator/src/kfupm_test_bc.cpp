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

    // Subscriber: still just prints odometry
    sub_ = nh.subscribe(odom_main_topic, 1, &KfupmTest::odometryCallback, this);

    // Publisher: unused, kept for completeness
    pub_ = nh.advertise<std_msgs::String>("output_topic", 1);

    // Service client for position control (Vec4 service)
    srv_client_control = nh.serviceClient<mrs_msgs::Vec4>(control_service);

    // NEW: wait for the control service to be available (optional but safer)
    NODELET_INFO("[KfupmTest]: waiting for control service '%s'...", control_service.c_str());
    srv_client_control.waitForExistence();
    NODELET_INFO("[KfupmTest]: control service is available.");

    // NEW: initialize step index (which part of the mission we are on)
    current_step_ = 0;
    mission_done_ = false;

    // Timer: fires every 5 seconds and advances the mission
    timer_ = nh.createTimer(ros::Duration(5.0), &KfupmTest::timerCallback, this, false);

    NODELET_INFO("KfupmTest initialized!");
  }

private:
  void odometryCallback(const nav_msgs::Odometry& msg)
  {
    ROS_INFO_THROTTLE(0.5, "UAV is at coordinates: x: %f, y: %f",
                      msg.pose.pose.position.x, msg.pose.pose.position.y);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (mission_done_)
    {
      // We already executed all steps; nothing else to do.
      return;
    }

    mrs_msgs::Vec4 position_reference;

    // Each call to this callback sends ONE step of the mission.
    // We assume the control_service is a *relative* goto service
    // (e.g. /uav1/control_manager/goto_relative) with:
    //   goal = [dx, dy, dz, heading]
    // in a frame where x=forward, y=left, z=up.

    switch (current_step_)
    {
      case 0:
        NODELET_INFO("[KfupmTest]: initialization");
        break;
      case 1:
        // Step 1: move forward 3.5 m
        NODELET_INFO("[KfupmTest]: Step 1 - moving forward 3.5 m");
        position_reference.request.goal[0] = 3.5;  // dx (forward)
        position_reference.request.goal[1] = 0.0;  // dy
        position_reference.request.goal[2] = 0.0;  // dz
        position_reference.request.goal[3] = 0.0;  // heading (no change)
        break;

      case 2:
        // Step 2: from new position, move (2, 3) relative
        NODELET_INFO("[KfupmTest]: Step 2 - moving (2, 3) m relative");
        position_reference.request.goal[0] = 2.0;  // +2 m in x (forward)
        position_reference.request.goal[1] = 3.0;  // +3 m in y (left)
        position_reference.request.goal[2] = 0.0;
        position_reference.request.goal[3] = 0.0;
        break;

      case 3:
        // Step 3: move back 5 m (opposite of forward => negative x)
        NODELET_INFO("[KfupmTest]: Step 3 - moving back 5 m");
        position_reference.request.goal[0] = -5.0; // -5 m in x (backward)
        position_reference.request.goal[1] = 0.0;
        position_reference.request.goal[2] = 0.0;
        position_reference.request.goal[3] = 0.0;
        break;

      case 4:
        // Step 4: move right 2 m (right = negative y in ENU convention)
        NODELET_INFO("[KfupmTest]: Step 4 - moving right 2 m");
        position_reference.request.goal[0] = 0.0;
        position_reference.request.goal[1] = -2.0; // -2 m in y (right)
        position_reference.request.goal[2] = 0.0;
        position_reference.request.goal[3] = 0.0;
        break;

      default:
        // All steps done: stop sending commands
        NODELET_INFO("[KfupmTest]: Mission completed, stopping timer.");
        mission_done_ = true;
        timer_.stop();
        return;
    }

    // Call the control service and check if it succeeded
    if (srv_client_control.call(position_reference))
    {
      if (position_reference.response.success)
      {
        NODELET_INFO("[KfupmTest]: Step %d command accepted: %s",
                     current_step_ + 1,
                     position_reference.response.message.c_str());
      }
      else
      {
        NODELET_WARN("[KfupmTest]: Step %d command rejected: %s",
                     current_step_ + 1,
                     position_reference.response.message.c_str());
      }
    }
    else
    {
      NODELET_ERROR("[KfupmTest]: Failed to call control service '%s' for step %d",
                    control_service.c_str(), current_step_ + 1);
    }

    // Advance to next step for the next timer tick
    current_step_++;
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
    
  ros::ServiceClient srv_client_control;
  
  std::string uav_name;
  std::string odom_main_topic;
  std::string control_service;

  // NEW: internal state for multi-step mission
  int  current_step_;
  bool mission_done_;
};

} // namespace kfupm_test

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kfupm_test::KfupmTest, nodelet::Nodelet)
