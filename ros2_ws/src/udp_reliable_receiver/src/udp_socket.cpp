#include "udp_reliable_common/udp_socket.hpp"

#include <cstring>

namespace udp_reliable {

UdpSocket::UdpSocket() : fd_(-1) {}

UdpSocket::~UdpSocket() {
  closeSocket();
}

bool UdpSocket::openSocket() {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  return fd_ >= 0;
}

bool UdpSocket::bindPort(int port) {
  if (fd_ < 0) {
    return false;
  }

  sockaddr_in addr{};
  std::memset(&addr, 0, sizeof(addr));

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(static_cast<uint16_t>(port));

  return ::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0;
}

bool UdpSocket::setReceiveTimeoutMs(int timeout_ms) {
  if (fd_ < 0) {
    return false;
  }

  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  return ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
}

ssize_t UdpSocket::sendTo(const void *data, size_t size, const sockaddr_in &addr) {
  return ::sendto(
    fd_,
    data,
    size,
    0,
    reinterpret_cast<const sockaddr *>(&addr),
    sizeof(addr)
  );
}

ssize_t UdpSocket::recvFrom(void *buffer, size_t size, sockaddr_in &addr, socklen_t &addr_len) {
  return ::recvfrom(
    fd_,
    buffer,
    size,
    0,
    reinterpret_cast<sockaddr *>(&addr),
    &addr_len
  );
}

int UdpSocket::fd() const {
  return fd_;
}

void UdpSocket::closeSocket() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace udp_reliable
