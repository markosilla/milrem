#pragma once

#include <rclcpp/node.hpp>

#include "messages/msg/data.hpp"

namespace udp {
  class Udp : public rclcpp::Node {
  public:
    Udp(const std::string &nodeName,
        const rclcpp::NodeOptions &options);

  private:
    const int QOS = 10;

    rclcpp::TimerBase::SharedPtr publishTimer_;
    rclcpp::Publisher<messages::msg::Data>::SharedPtr publisher_;

    void initialize();

    void publishData();

    messages::msg::Data getAveragedData();
  };
}  // namespace udp
