#pragma once

#include <arpa/inet.h>
#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace udp_reliable {

class UdpSocket {
public:
  UdpSocket();
  ~UdpSocket();

  bool openSocket();
  bool bindPort(int port);
  bool setReceiveTimeoutMs(int timeout_ms);

  ssize_t sendTo(const void *data, size_t size, const sockaddr_in &addr);
  ssize_t recvFrom(void *buffer, size_t size, sockaddr_in &addr, socklen_t &addr_len);

  int fd() const;
  void closeSocket();

private:
  int fd_;
};

}  // namespace udp_reliable
