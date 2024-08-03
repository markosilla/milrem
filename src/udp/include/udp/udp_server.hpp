#pragma once

#include <string>
#include <netinet/in.h>

namespace udp
{
  class UdpServer
  {
  public:
    UdpServer(uint16_t sendPort, uint16_t recvPort);
    ~UdpServer();

    void init();
    void exit();
    void sendData(const std::string &message, const std::string &address);

  private:
    int createSocket();
    int createEpoll();
    void eventLoop();
    void handleIncomingData(int fd);

    uint16_t sendPort;
    uint16_t recvPort;

    int socketFd;
    int epollFd;

    struct sockaddr_in serverAddr;
    struct sockaddr_in sendAddr;
  };
} // namespace udp