#pragma once

#include <rclcpp/node.hpp>

#include "udp/packet.hpp"
#include "udp/udp_server.hpp"
#include "messages/msg/data.hpp"

namespace udp
{
  class Udp : public rclcpp::Node
  {
  public:
    Udp(const std::string &nodeName, const rclcpp::NodeOptions &options);
    ~Udp();

  private:
    const int QOS = 10;

    rclcpp::TimerBase::SharedPtr publishTimer_;
    rclcpp::Publisher<messages::msg::Data>::SharedPtr publisher_;
    UdpServer *server_;
    common::RingBuffer<Packet> *ring_buffer_;

    void initialize();

    void publishData();

    messages::msg::Data getAveragedData();
    
    std::thread calculateAveragesThread;
    void calculateAverages();
  };
} // namespace udp
