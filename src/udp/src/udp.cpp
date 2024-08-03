#include <chrono>
#include <time.h>
#include <unistd.h>

#include "udp/udp.hpp"
#include "udp/udp_server.hpp"

namespace udp {
  Udp::Udp(const std::string &nodeName,
           const rclcpp::NodeOptions &options) :
      Node(nodeName, options) {
    const auto frequency = get_parameter("data_publish_hz").as_double();
    const auto interval = std::chrono::milliseconds(static_cast<int>(1e3 / frequency));

    const auto publisher_ = create_publisher<messages::msg::Data>("udp/data", QOS);
    const auto publishTimer_ = create_wall_timer(interval, [this]() { publishData(); });

    initialize();

    std::cout << "UDP started\n";
  }

  void Udp::initialize() {
    // TODO: configure UDP socket
    //  * Start receiving data. Store all received data, up to some reasonable number.
    //    Note: Data should be stored in a circular buffer. For this, implement a
    //    Ring Buffer class. Data must be stored in the type defined in UDP packet.
    //  * Start sending pinging packets and calculate connection latency. Print
    //    latency value (in μs) with a 5s interval.
    //  * UDP communication must not block the main thread.

    std::shared_ptr<udp::UdpServer> server;
    server = std::make_shared<udp::UdpServer>(12346, 12345);
    server->init();
  }

  messages::msg::Data Udp::getAveragedData() {
    // TODO: Average (mean) received data over the last N seconds (defined in params)
    //  for each sensor. Also add a timestamp corresponding to the time of averaging.
    return messages::msg::Data();
  }

  void Udp::publishData() {
    publisher_->publish(getAveragedData());
  }
}  // namespace udp
