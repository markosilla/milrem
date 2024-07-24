#include <rclcpp/rclcpp.hpp>

#include "udp/udp.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<udp::Udp>("udp_node", options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
