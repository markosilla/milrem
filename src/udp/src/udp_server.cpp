#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ctime>
#include <thread>

#include "udp/packet.hpp"
#include "udp/udp_server.hpp"

namespace udp
{
  UdpServer::UdpServer(int sendPort, int recvPort, common::RingBuffer<Packet> *ring_buffer)
      : sendPort(sendPort), recvPort(recvPort), ring_buffer(ring_buffer)
  {
    memset(&recvAddr, 0, sizeof(recvAddr));
    memset(&sendAddr, 0, sizeof(sendAddr));
    socketFd = -1;
    epollFd = -1;
  }

  UdpServer::~UdpServer()
  {
    stop();
    shutdown();
  }

  int UdpServer::createSocket()
  {
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd == -1)
    {
      std::cerr << "Socket creation error: " << strerror(errno) << std::endl;
      return -errno;
    }

    recvAddr.sin_family = AF_INET;
    recvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    recvAddr.sin_port = htons(recvPort);

    sendAddr.sin_family = AF_INET;
    sendAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    sendAddr.sin_port = htons(sendPort);

    if (bind(sd, (struct sockaddr *)&recvAddr, sizeof(recvAddr)) == -1)
    {
      std::cerr << "Bind error: " << strerror(errno) << std::endl;
      close(sd);
      return -errno;
    }

    return sd;
  }

  int UdpServer::createEpoll()
  {
    struct epoll_event event;
    int efd = epoll_create1(0);

    if (efd == -1)
    {
      std::cerr << "Epoll creation error: " << strerror(errno) << std::endl;
      return -errno;
    }

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

  void UdpServer::setup()
  {
    socketFd = createSocket();
    if (socketFd == -1)
    {
      std::cerr << "Failed to create socket" << std::endl;
      shutdown();
      return;
    }

    epollFd = createEpoll();
    if (epollFd == -1)
    {
      std::cerr << "Failed to setup epoll" << std::endl;
      shutdown();
      return;
    }
  }

  void UdpServer::start()
  {
    std::thread eventLoopThread(&UdpServer::eventLoop, this);
    std::thread sendPingThread(&UdpServer::sendPing, this);
    
    eventLoopThread.detach();
    sendPingThread.detach();
  }

  void UdpServer::stop()
  {
    if (eventLoopThread.joinable())
    {
      eventLoopThread.join();
    }
    if (sendPingThread.joinable())
    {
      sendPingThread.join();
    }
  }

  void UdpServer::shutdown()
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
    ssize_t nBytes = recvfrom(fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&src_addr, &src_addr_len);
    if (nBytes == -1)
    {
      std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
      return;
    }

    if (nBytes == 4 && std::strncmp(buffer, "pong", 4) == 0)
    {
      pongReceivedTime_ = std::chrono::steady_clock::now();
      std::cout << "Latency: " << std::chrono::duration_cast<std::chrono::microseconds>(pongReceivedTime_ - pingSentTime_).count() << " μs\n";
      return;
    }

    packet = buffer;
    ring_buffer->put(packet);

    static int counter = 1;    
    std::cout << "Nof packets received: " << counter++ << std::endl;
  }

  void UdpServer::sendPing()
  {
    char ping[] = "ping";
    ssize_t nBytes;

    while (true)
    {
      nBytes = sendto(socketFd, ping, 4, 0, (struct sockaddr *)&sendAddr, sizeof(sendAddr));
      if (nBytes == -1)
      {
        std::cerr << "sendto error: " << strerror(errno) << std::endl;
        return;
      }

      pingSentTime_ = std::chrono::steady_clock::now();
      std::cout << "Sent ping: " << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) << "\n";
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
  }

} // namespace udp