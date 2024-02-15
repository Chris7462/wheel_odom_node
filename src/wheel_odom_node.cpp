#include "wheel_odom/wheel_odom.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_odom_node::WheelOdomNode>());
  rclcpp::shutdown();

  return 0;
}
