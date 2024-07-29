#include <chrono>
#include <time.h>
#include <unistd.h>
#include <thread>
#include <unordered_map>

#include "udp/udp.hpp"
#include "udp/udp_server.hpp"
#include "udp/packet.hpp"
#include "common/ring_buffer.hpp"

namespace udp {
  Udp::Udp(const std::string &nodeName,
           const rclcpp::NodeOptions &options) :
      Node(nodeName, options) {
    const auto frequency = get_parameter("data_publish_hz").as_double();
    const auto interval = std::chrono::milliseconds(static_cast<int>(1e3 / frequency));
    const auto receive_port = get_parameter("receive_port").as_int();
    const auto send_port = get_parameter("send_port").as_int();

    publisher_ = create_publisher<messages::msg::Data>("udp/data", QOS);
    publishTimer_ = create_wall_timer(interval, [this]() { publishData(); });
    ring_buffer_ = new common::RingBuffer<Packet>(100);
    server_ = new UdpServer(send_port, receive_port, ring_buffer_);
    
    initialize();

    std::cout << "UDP started\n";
  }

  Udp::~Udp() {
    if (!server_)
      return;

    server_->stop();
    server_->shutdown();

    if (calculateAveragesThread.joinable())
    {
      calculateAveragesThread.join();
    }

    delete ring_buffer_;

    std::cout << "UDP stopped\n";
  }

  void Udp::calculateAverages()
  {
    std::unordered_map<uint8_t, int> packetsPerSensor;
    std::unordered_map<uint8_t, double> sumPerSensor;

    while (true) {
        Packet packet = ring_buffer_->get();

        packetsPerSensor[packet.sensorId]++;
        sumPerSensor[packet.sensorId] += packet.getValueAsDouble();

        double average = sumPerSensor[packet.sensorId] / packetsPerSensor[packet.sensorId];

        std::cout << "Sensor " << static_cast<int>(packet.sensorId) << " packet count: " << packetsPerSensor[packet.sensorId]
                  << ", average value: " << average << std::endl;
    }
  }

  void Udp::initialize() {
    // TODO: configure UDP socket
    //  * Start receiving data. Store all received data, up to some reasonable number.
    //    Note: Data should be stored in a circular buffer. For this, implement a
    //    Ring Buffer class. Data must be stored in the type defined in UDP packet.
    //  * Start sending pinging packets and calculate connection latency. Print
    //    latency value (in μs) with a 5s interval.
    //  * UDP communication must not block the main thread.

    server_->setup();
    server_->start();

    std::thread calculateAveragesThread(&Udp::calculateAverages, this); 

    calculateAveragesThread.detach();
  }

  messages::msg::Data Udp::getAveragedData() {
    // const auto average_duration_s = get_parameter("average_duration_s").as_int();




    return messages::msg::Data();
    // TODO: Average (mean) received data over the last N seconds (defined in params)
    //  for each sensor. Also add a timestamp corresponding to the time of averaging.
  }

  void Udp::publishData() {
    publisher_->publish(getAveragedData());
  }
}  // namespace udp
