// ros header
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// local header
#include "wheel_odom/wheel_odom.hpp"


namespace wheel_odom_node
{

WheelOdomNode::WheelOdomNode()
: Node("wheel_odom_node"), sync_(policy_t(10), sub_wheel_spd_, sub_imu_)
{
  rclcpp::QoS qos(10);

  // sync gps and imu msg
  auto rmw_qos_profile = qos.get_rmw_qos_profile();
  sub_wheel_spd_.subscribe(this, "kitti/oxts/gps/vel", rmw_qos_profile);
  sub_imu_.subscribe(this, "kitti/oxts/imu", rmw_qos_profile);
  sync_.registerCallback(&WheelOdomNode::sync_callback, this);

  pub_wheel_odom_ = create_publisher<nav_msgs::msg::Odometry>(
    "wheel_odom", qos);
}

void WheelOdomNode::sync_callback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr wheel_spd_msg,
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  tf2::Quaternion q_current;
  tf2::fromMsg(imu_msg->orientation, q_current);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);
  q_current.setRPY(0.0, 0.0, wrap2pi(yaw));
  q_current.normalize();

  RCLCPP_INFO(get_logger(), "Roll  = %f", roll);
  RCLCPP_INFO(get_logger(), "Pitch = %f", pitch);
  RCLCPP_INFO(get_logger(), "Yaw   = %f", yaw);

  nav_msgs::msg::Odometry wheel_odom_msg;
  wheel_odom_msg.header.stamp = rclcpp::Node::now();
  wheel_odom_msg.header.frame_id = "odom";
  wheel_odom_msg.child_frame_id = "base_link";
//wheel_odom_msg.pose.pose.position;  // x, y, z
  wheel_odom_msg.pose.pose.orientation = tf2::toMsg(q_current);
//wheel_odom_msg.pose.covariance; // x, y, z, roll, pitch, yaw
//wheel_odom_msg.twist.twist.linear;  // x, y, z;
//wheel_odom_msg.twist.twist.angular; // x, y, z;
//wheel_odom_msg.twist.covariance;  // x, y, z, roll, pitch, yaw
}

double WheelOdomNode::wrap2pi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace wheel_odom_node
