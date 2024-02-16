#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace wheel_odom_node
{

class WheelOdomNode : public rclcpp::Node
{
public:
  WheelOdomNode();
  ~WheelOdomNode() = default;

private:
  message_filters::Subscriber<geometry_msgs::msg::TwistStamped> sub_wheel_spd_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> sub_imu_;

  using policy_t = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::TwistStamped, sensor_msgs::msg::Imu>;

  message_filters::Synchronizer<policy_t> sync_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;

  void sync_callback(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr wheel_spd_msg,
    const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

  double wrap2pi(const double angle);

  bool init_;
  double pose_x_;
  double pose_y_;
  double previous_time_;
};

} // wheel_odom_node
