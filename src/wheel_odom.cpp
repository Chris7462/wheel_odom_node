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
}

void WheelOdomNode::sync_callback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr wheel_spd_msg,
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  RCLCPP_INFO(get_logger(), "in the callback");
}

} // namespace wheel_odom_node
