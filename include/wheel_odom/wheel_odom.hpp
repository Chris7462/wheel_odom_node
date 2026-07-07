#pragma once

// std header
#include <optional>

// ros header
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace wheel_odom
{

class WheelOdom : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for WheelOdomNode
   */
  WheelOdom();

  /**
   * @brief Destructor for WheelOdomNode
   */
  ~WheelOdom() = default;

private:
  /**
   * @brief Initialize node parameters with validation
   * @return true if initialization successful, false otherwise
   */
  bool initialize_parameters();

  /**
   * @brief Initialize ROS2 publishers, subscribers, and message filters
   * @return true if initialization successful, false otherwise
   */
  bool initialize_ros_components();

  /**
   * @brief Synchronized callback for incoming wheel speed and IMU messages
   * @param wheel_spd_msg Incoming wheel speed message
   * @param imu_msg Incoming IMU message
   */
  void sync_callback(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr wheel_spd_msg,
    const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

  /**
   * @brief Wrap an angle to the range [-pi, pi]
   */
  double wrap2pi(const double angle);

private:
  // ROS2 parameters: topics
  std::string wheel_spd_input_topic_;
  std::string imu_input_topic_;
  std::string wheel_odom_output_topic_;

  // ROS2 parameters: queues
  int queue_size_;
  int sync_queue_size_;

  // Message filter subscribers
  message_filters::Subscriber<geometry_msgs::msg::TwistStamped> sub_wheel_spd_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> sub_imu_;

  using policy_t = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::TwistStamped, sensor_msgs::msg::Imu>;

  std::shared_ptr<message_filters::Synchronizer<policy_t>> sync_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;

  double pose_x_;
  double pose_y_;
  std::optional<double> previous_time_;
};

} // wheel_odom
