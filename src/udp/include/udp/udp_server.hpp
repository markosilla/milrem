#pragma once

#include <string>
#include <netinet/in.h>
#include <chrono>

#include "udp/packet.hpp"
#include "common/ring_buffer.hpp"

namespace udp
{
  class UdpServer
  {
  public:
    UdpServer(int sendPort, int recvPort, common::RingBuffer<Packet> *ring_buffer);
    ~UdpServer();

    void setup();
    void shutdown();

    void start();
    void stop();

  private:

    int createSocket();
    int createEpoll();

    /* threads */
    std::thread eventLoopThread;
    std::thread sendPingThread;

    void eventLoop();
    void sendPing();

    void handleIncomingData(int fd);

    uint16_t sendPort;
    uint16_t recvPort;
    common::RingBuffer<Packet> *ring_buffer;

    int socketFd;
    int epollFd;

    struct sockaddr_in sendAddr;
    struct sockaddr_in recvAddr;

    std::chrono::steady_clock::time_point pingSentTime_;
    std::chrono::steady_clock::time_point pongReceivedTime_;
  };
} // namespace udp