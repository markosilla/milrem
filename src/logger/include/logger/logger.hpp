#pragma once

#include <rclcpp/node.hpp>

#include "messages/msg/data.hpp"

namespace logger {
  class Logger : public rclcpp::Node {
  public:
    Logger(const std::string &nodeName,
           const rclcpp::NodeOptions &options);

  private:
    const int QOS = 10;

    rclcpp::Subscription<messages::msg::Data>::SharedPtr subscription_;

    void initialize();
  };
}  // namespace logger
