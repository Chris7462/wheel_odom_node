// ros header
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// local header
#include "wheel_odom/wheel_odom.hpp"


namespace wheel_odom
{

WheelOdom::WheelOdom()
: Node("wheel_odom_node"),
  pose_x_{0.0}, pose_y_{0.0}, previous_time_{std::nullopt}
{
  // Initialize ROS2 parameters with validation
  if (!initialize_parameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize parameters");
    rclcpp::shutdown();
    return;
  }

  // Initialize ROS2 publishers, subscribers, and message filters
  if (!initialize_ros_components()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize ROS components");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "Wheel odom node initialized successfully");
}

bool WheelOdom::initialize_parameters()
{
  try {
    // Input topics
    wheel_spd_input_topic_ = declare_parameter(
      "wheel_spd_input_topic", std::string("kitti/oxts/gps/vel"));
    imu_input_topic_ = declare_parameter(
      "imu_input_topic", std::string("kitti/vehicle/imu"));

    // Output topic
    wheel_odom_output_topic_ = declare_parameter(
      "wheel_odom_output_topic", std::string("kitti/wheel_odom"));

    // Queue sizes
    queue_size_ = declare_parameter<int>("queue_size", 10);
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);

    if (queue_size_ <= 0 || sync_queue_size_ <= 0) {
      RCLCPP_ERROR(
        get_logger(), "Invalid queue sizes: queue_size=%d, sync_queue_size=%d",
        queue_size_, sync_queue_size_);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Parameters initialized successfully");
    RCLCPP_INFO(
      get_logger(), "Input topics: %s, %s",
      wheel_spd_input_topic_.c_str(), imu_input_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Output topic: %s", wheel_odom_output_topic_.c_str());

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during parameter initialization: %s", e.what());
    return false;
  }
}

bool WheelOdom::initialize_ros_components()
{
  try {
    rclcpp::QoS qos(queue_size_);
    // Wheel speed and IMU are low-rate, safety-relevant signals -- explicitly reliable,
    // matching local_gps_imu's convention.
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.history(rclcpp::HistoryPolicy::KeepLast);

    // sync wheel speed and imu msg. NOTE: message_filters::Subscriber::subscribe() takes
    // an rclcpp::QoS directly as of Jazzy/Lyrical -- the old pattern of converting to
    // qos.get_rmw_qos_profile() no longer matches any overload and fails to build.
    sub_wheel_spd_.subscribe(this, wheel_spd_input_topic_, qos);
    sub_imu_.subscribe(this, imu_input_topic_, qos);

    sync_ = std::make_shared<message_filters::Synchronizer<policy_t>>(
      policy_t(sync_queue_size_), sub_wheel_spd_, sub_imu_);
    sync_->registerCallback(&WheelOdom::sync_callback, this);

    pub_wheel_odom_ = create_publisher<nav_msgs::msg::Odometry>(
      wheel_odom_output_topic_, qos);

    RCLCPP_INFO(get_logger(), "ROS components initialized successfully");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during ROS component initialization: %s", e.what());
    return false;
  }
}

void WheelOdom::sync_callback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr wheel_spd_msg,
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  double current_time = rclcpp::Time(wheel_spd_msg->header.stamp).seconds();

  // dt is 0 on the first message (no previous timestamp yet), so integration
  // is a no-op that call
  double dt = previous_time_ ? (current_time - *previous_time_) : 0.0;
  previous_time_ = current_time;

  // calculate the current pose of the vehicle based on the wheel speed
  const double v = wheel_spd_msg->twist.linear.x;
  const double a = imu_msg->linear_acceleration.x;
  double yaw, dummy;
  tf2::getEulerYPR(imu_msg->orientation, yaw, dummy, dummy);
  pose_x_ += (v * dt + 0.5 * a * dt * dt) * std::cos(yaw);
  pose_y_ += (v * dt + 0.5 * a * dt * dt) * std::sin(yaw);

  tf2::Quaternion q_current;
  q_current.setRPY(0.0, 0.0, wrap2pi(yaw));
  q_current.normalize();

  // publish wheel odom msg
  nav_msgs::msg::Odometry wheel_odom_msg;
  wheel_odom_msg.header.stamp = wheel_spd_msg->header.stamp;
  wheel_odom_msg.header.frame_id = "odom";
  wheel_odom_msg.child_frame_id = "base_link";
  tf2::toMsg(tf2::Vector3(pose_x_, pose_y_, 0.0), wheel_odom_msg.pose.pose.position);  // x, y, z
  wheel_odom_msg.pose.pose.orientation = tf2::toMsg(q_current);
  //wheel_odom_msg.pose.covariance; // x, y, z, roll, pitch, yaw
  wheel_odom_msg.twist.twist.linear = wheel_spd_msg->twist.linear;
  wheel_odom_msg.twist.twist.angular = imu_msg->angular_velocity;
  //wheel_odom_msg.twist.covariance;  // x, y, z, roll, pitch, yaw
  pub_wheel_odom_->publish(wheel_odom_msg);
}

double WheelOdom::wrap2pi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace wheel_odom_node
