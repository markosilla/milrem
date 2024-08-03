#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "udp/packet.hpp"
#include "udp/udp_server.hpp"

namespace udp
{
  UdpServer::UdpServer(uint16_t sendPort, uint16_t recvPort)
      : sendPort(sendPort), recvPort(recvPort), socketFd(-1), epollFd(-1)
  {
    memset(&serverAddr, 0, sizeof(serverAddr));
    memset(&sendAddr, 0, sizeof(sendAddr));
  }

  UdpServer::~UdpServer() { exit(); }

  int UdpServer::createSocket()
  {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd == -1)
    {
      std::cerr << "Socket creation error: " << strerror(errno) << std::endl;
      return -errno;
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_port = htons(recvPort);

    if (bind(sd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1)
    {
      std::cerr << "Bind error: " << strerror(errno) << std::endl;
      close(sd);
      return -errno;
    }

    return sd;
  }

  int UdpServer::createEpoll()
  {
    int efd = epoll_create1(0);
    if (efd == -1)
    {
      std::cerr << "Epoll creation error: " << strerror(errno) << std::endl;
      return -errno;
    }

    struct epoll_event event;
    event.events = EPOLLIN;
    event.data.fd = socketFd;

    if (epoll_ctl(efd, EPOLL_CTL_ADD, socketFd, &event) == -1)
    {
      std::cerr << "Epoll control error: " << strerror(errno) << std::endl;
      close(efd);
      return -errno;
    }

    return efd;
  }

  void UdpServer::init()
  {
    socketFd = createSocket();
    if (socketFd == -1)
    {
      std::cerr << "Failed to create socket" << std::endl;
      exit();
      return;
    }

    epollFd = createEpoll();
    if (epollFd == -1)
    {
      std::cerr << "Failed to setup epoll" << std::endl;
      exit();
      return;
    }

    eventLoop();
  }

  void UdpServer::exit()
  {
    if (socketFd != -1)
    {
      close(socketFd);
      socketFd = -1;
    }
    if (epollFd != -1)
    {
      close(epollFd);
      epollFd = -1;
    }
  }

  void UdpServer::eventLoop()
  {
    const int maxEvents = 16;
    struct epoll_event events[maxEvents];
    while (true)
    {
      int nofEvents = epoll_wait(epollFd, events, maxEvents, -1);
      for (int i = 0; i < nofEvents; ++i)
      {
        if (events[i].events & EPOLLIN)
        {
          handleIncomingData(events[i].data.fd);
        }
      }
    }
  }

  void UdpServer::handleIncomingData(int fd)
  {
    char buffer[15] = {0};
    Packet packet;
    struct sockaddr_storage src_addr;
    socklen_t src_addr_len = sizeof(src_addr);

    ssize_t nBytes = recvfrom(fd, buffer, sizeof(buffer), 0,
                              (struct sockaddr *)&src_addr, &src_addr_len);
    if (nBytes == -1)
    {
      std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
      return;
    }

    // std::memcpy(&packet, buffer, sizeof(Packet));
    packet = buffer;

    std::cout << packet.toString() << std::endl;
  }

  void UdpServer::sendData(const std::string &message,
                           const std::string &address)
  {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
    {
      std::cerr << "Socket creation error: " << strerror(errno) << std::endl;
      return;
    }

    sendAddr.sin_family = AF_INET;
    sendAddr.sin_addr.s_addr = inet_addr(address.c_str());
    sendAddr.sin_port = htons(sendPort);

    ssize_t count = sendto(sock, message.c_str(), message.size(), 0,
                           (struct sockaddr *)&sendAddr, sizeof(sendAddr));
    if (count == -1)
    {
      std::cerr << "sendto error: " << strerror(errno) << std::endl;
    }

    close(sock);
  }

} // namespace udp