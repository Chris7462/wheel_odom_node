// ros header
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// local header
#include "wheel_odom/wheel_odom.hpp"


namespace wheel_odom_node
{

WheelOdomNode::WheelOdomNode()
: Node("wheel_odom_node"), sync_(policy_t(10), sub_wheel_spd_, sub_imu_),
  init_{false}, pose_x_{0.0}, pose_y_{0.0}, previous_time_{0.0}
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
  double dt;
  if (!init_) {
    dt = 0.0;
    init_ = true;
  } else {
    dt = rclcpp::Time(wheel_spd_msg->header.stamp).seconds() - previous_time_;
  }

  tf2::Quaternion q_current;
  tf2::fromMsg(imu_msg->orientation, q_current);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);
  q_current.setRPY(0.0, 0.0, wrap2pi(yaw));
  q_current.normalize();

  // calculate the current pose of the vehicle
  const double & v = wheel_spd_msg->twist.linear.x;
  const double & a = imu_msg->linear_acceleration.x;
  pose_x_ += (v * dt + 0.5 * a * dt * dt) * std::cos(yaw);
  pose_y_ += (v * dt + 0.5 * a * dt * dt) * std::sin(yaw);

  nav_msgs::msg::Odometry wheel_odom_msg;
  wheel_odom_msg.header.stamp = wheel_spd_msg->header.stamp;
  wheel_odom_msg.header.frame_id = "odom";
  wheel_odom_msg.child_frame_id = "base_link";
  tf2::toMsg(tf2::Vector3(pose_x_, pose_y_, 0.0), wheel_odom_msg.pose.pose.position);  // x, y, z
  wheel_odom_msg.pose.pose.orientation = tf2::toMsg(q_current);
//wheel_odom_msg.pose.covariance; // x, y, z, roll, pitch, yaw
//wheel_odom_msg.twist.twist.linear;  // x, y, z;
//wheel_odom_msg.twist.twist.angular; // x, y, z;
//wheel_odom_msg.twist.covariance;  // x, y, z, roll, pitch, yaw

  previous_time_ = rclcpp::Time(wheel_spd_msg->header.stamp).seconds();
}

double WheelOdomNode::wrap2pi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace wheel_odom_node
