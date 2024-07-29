#include <chrono>

#include "logger/logger.hpp"

namespace logger {
  Logger::Logger(const std::string &nodeName,
                 const rclcpp::NodeOptions &options) :
      Node(nodeName, options) {
    subscription_ = create_subscription<messages::msg::Data>("udp/data", QOS,
        [&](messages::msg::Data::SharedPtr msg) {
          (void)msg;
          // TODO: store received data to a FIFO buffer
        });

    initialize();

    std::cout << "Logger started\n";
  }

  void Logger::initialize() {
    const auto frequency = get_parameter("data_log_hz").as_double();
    const auto interval = std::chrono::milliseconds(static_cast<int>(1e3 / frequency));
    const auto filePath = get_parameter("csv_path").as_string();

    // TODO: Periodically save all new received data to a local csv file and remove
    //  the written data from the FIFO buffer.
  }
}  // namespace logger
