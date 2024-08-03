#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

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

  union Value
  {
    bool value_bool;
    int8_t value_int8;
    int16_t value_int16;
    int32_t value_int32;
    int64_t value_int64;
    uint8_t value_uint8;
    uint16_t value_uint16;
    uint32_t value_uint32;
    uint64_t value_uint64;
    float value_float;
    double value_double;
    char value_char;
  };

  struct __attribute__((__packed__)) Packet
  {
    uint8_t sensorId;
    uint8_t dataType;
    Value value;

    std::string toString() const
    {
      std::ostringstream oss;
      oss << "Sensor ID: " << static_cast<int>(sensorId)
          << ", Data Type: " << static_cast<int>(dataType) << ", Value: ";
      switch (dataType)
      {
      case 0:
        oss << value.value_bool;
        break;
      case 1:
        oss << static_cast<int>(value.value_int8);
        break;
      case 2:
        oss << value.value_int16;
        break;
      case 3:
        oss << value.value_int32;
        break;
      case 4:
        oss << value.value_int64;
        break;
      case 5:
        oss << static_cast<unsigned int>(value.value_uint8);
        break;
      case 6:
        oss << value.value_uint16;
        break;
      case 7:
        oss << value.value_uint32;
        break;
      case 8:
        oss << value.value_uint64;
        break;
      case 9:
        oss << value.value_float;
        break;
      case 10:
        oss << value.value_double;
        break;
      case 11:
        oss << value.value_char;
        break;
      default:
        oss << "Unknown";
        break;
      }
      return oss.str();
    }

    Packet &operator=(const char *buffer)
    {
      size_t offset = 0;

      sensorId = buffer[offset];
      offset += sizeof(sensorId);

      dataType = buffer[offset];
      offset += sizeof(dataType);

      switch (dataType)
      {
      case 0:
        value.value_bool = buffer[offset];
        break;
      case 1:
        value.value_int8 = buffer[offset];
        break;
      case 2:
        std::memcpy(&value.value_int16, buffer + offset,
                    sizeof(value.value_int16));
        break;
      case 3:
        std::memcpy(&value.value_int32, buffer + offset,
                    sizeof(value.value_int32));
        break;
      case 4:
        std::memcpy(&value.value_int64, buffer + offset,
                    sizeof(value.value_int64));
        break;
      case 5:
        value.value_uint8 = buffer[offset];
        break;
      case 6:
        std::memcpy(&value.value_uint16, buffer + offset,
                    sizeof(value.value_uint16));
        break;
      case 7:
        std::memcpy(&value.value_uint32, buffer + offset,
                    sizeof(value.value_uint32));
        break;
      case 8:
        std::memcpy(&value.value_uint64, buffer + offset,
                    sizeof(value.value_uint64));
        break;
      case 9:
        std::memcpy(&value.value_float, buffer + offset,
                    sizeof(value.value_float));
        break;
      case 10:
        std::memcpy(&value.value_double, buffer + offset,
                    sizeof(value.value_double));
        break;
      case 11:
        value.value_char = buffer[offset];
        break;
      default:
        std::cerr << "Unknown data type" << std::endl;
        break;
      }

      return *this;
    }
  };

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