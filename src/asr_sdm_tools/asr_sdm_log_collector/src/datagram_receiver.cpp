#include "asr_sdm_log_collector/datagram_receiver.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace asr_sdm::log
{

namespace
{

/// Bounds how long stop() waits for the receive thread to notice the flag.
constexpr int kPollTimeoutMilliseconds = 100;

std::string describeErrno(const std::string & context, int error_number)
{
  return context + ": " + std::strerror(error_number);
}

/// Removes a leftover socket file from a previous run so bind() can succeed.
/// Refuses to touch anything that is not a socket, so a mistyped path pointing
/// at a real file cannot destroy it.
void removeStaleSocket(const std::string & path)
{
  struct stat status
  {
  };
  if (::stat(path.c_str(), &status) != 0) {
    return;
  }
  if (!S_ISSOCK(status.st_mode)) {
    throw std::runtime_error("refusing to replace '" + path + "', it is not a socket");
  }
  ::unlink(path.c_str());
}

}  // namespace

DatagramReceiver::DatagramReceiver(
  const UdpEndpoint & endpoint, Options options, DatagramHandler handler)
: options_{options},
  handler_{std::move(handler)},
  description_{"udp " + endpoint.bind_address + ":" + std::to_string(endpoint.port)}
{
  if (!handler_) {
    throw std::invalid_argument("DatagramReceiver requires a datagram handler");
  }

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  hints.ai_flags = AI_PASSIVE;

  const std::string port_text = std::to_string(endpoint.port);
  addrinfo * resolved = nullptr;
  const int status =
    ::getaddrinfo(endpoint.bind_address.c_str(), port_text.c_str(), &hints, &resolved);
  if (status != 0 || resolved == nullptr) {
    throw std::runtime_error(
      "cannot resolve bind address '" + endpoint.bind_address + "': " + ::gai_strerror(status));
  }

  socket_ = ::socket(resolved->ai_family, resolved->ai_socktype, resolved->ai_protocol);
  if (socket_ < 0) {
    const int socket_errno = errno;
    ::freeaddrinfo(resolved);
    throw std::runtime_error(describeErrno("cannot create UDP socket", socket_errno));
  }

  const int reuse = 1;
  ::setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  applyCommonSocketOptions();

  if (::bind(socket_, resolved->ai_addr, resolved->ai_addrlen) != 0) {
    const int bind_errno = errno;
    ::freeaddrinfo(resolved);
    ::close(socket_);
    socket_ = -1;
    throw std::runtime_error(describeErrno("cannot bind " + description_, bind_errno));
  }
  ::freeaddrinfo(resolved);

  // Report the port the kernel actually assigned, which matters when port 0 was
  // requested to let the OS pick a free one.
  sockaddr_storage bound{};
  socklen_t bound_length = sizeof(bound);
  if (::getsockname(socket_, reinterpret_cast<sockaddr *>(&bound), &bound_length) == 0) {
    uint16_t bound_port = 0;
    if (bound.ss_family == AF_INET) {
      bound_port = ntohs(reinterpret_cast<sockaddr_in *>(&bound)->sin_port);
    } else if (bound.ss_family == AF_INET6) {
      bound_port = ntohs(reinterpret_cast<sockaddr_in6 *>(&bound)->sin6_port);
    }
    if (bound_port != 0) {
      description_ = "udp " + endpoint.bind_address + ":" + std::to_string(bound_port);
    }
  }
}

DatagramReceiver::DatagramReceiver(
  const UnixEndpoint & endpoint, Options options, DatagramHandler handler)
: options_{options}, handler_{std::move(handler)}, description_{"unix " + endpoint.socket_path}
{
  if (!handler_) {
    throw std::invalid_argument("DatagramReceiver requires a datagram handler");
  }

  sockaddr_un address{};
  if (endpoint.socket_path.empty() || endpoint.socket_path.size() >= sizeof(address.sun_path)) {
    throw std::runtime_error("unusable socket path '" + endpoint.socket_path + "'");
  }
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, endpoint.socket_path.c_str(), endpoint.socket_path.size());

  const std::filesystem::path socket_file{endpoint.socket_path};
  if (socket_file.has_parent_path()) {
    std::error_code error;
    std::filesystem::create_directories(socket_file.parent_path(), error);
    if (error && !std::filesystem::is_directory(socket_file.parent_path())) {
      throw std::runtime_error(
        "cannot create socket directory '" + socket_file.parent_path().string() +
        "': " + error.message());
    }
  }
  removeStaleSocket(endpoint.socket_path);

  socket_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  if (socket_ < 0) {
    throw std::runtime_error(describeErrno("cannot create Unix socket", errno));
  }
  applyCommonSocketOptions();

  if (::bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    const int bind_errno = errno;
    ::close(socket_);
    socket_ = -1;
    throw std::runtime_error(describeErrno("cannot bind " + description_, bind_errno));
  }
  owned_socket_path_ = endpoint.socket_path;

  // The mode has to be set after bind, since bind creates the socket file, and
  // the process umask would otherwise decide who may log.
  if (::chmod(endpoint.socket_path.c_str(), static_cast<mode_t>(endpoint.permissions)) != 0) {
    const int chmod_errno = errno;
    ::close(socket_);
    socket_ = -1;
    ::unlink(owned_socket_path_.c_str());
    owned_socket_path_.clear();
    throw std::runtime_error(
      describeErrno("cannot set permissions on " + endpoint.socket_path, chmod_errno));
  }
}

DatagramReceiver::~DatagramReceiver()
{
  stop();
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
  if (!owned_socket_path_.empty()) {
    ::unlink(owned_socket_path_.c_str());
  }
}

void DatagramReceiver::applyCommonSocketOptions()
{
  ::setsockopt(
    socket_, SOL_SOCKET, SO_RCVBUF, &options_.receive_buffer_bytes,
    sizeof(options_.receive_buffer_bytes));
}

void DatagramReceiver::start()
{
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&DatagramReceiver::receiveLoop, this);
}

void DatagramReceiver::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

const std::string & DatagramReceiver::description() const
{
  return description_;
}

uint64_t DatagramReceiver::receivedCount() const
{
  return received_.load(std::memory_order_relaxed);
}

uint64_t DatagramReceiver::errorCount() const
{
  return errors_.load(std::memory_order_relaxed);
}

void DatagramReceiver::receiveLoop()
{
  std::vector<char> buffer(options_.max_datagram_bytes == 0 ? 65536 : options_.max_datagram_bytes);

  while (running_.load(std::memory_order_relaxed)) {
    pollfd descriptor{};
    descriptor.fd = socket_;
    descriptor.events = POLLIN;

    const int ready = ::poll(&descriptor, 1, kPollTimeoutMilliseconds);
    if (ready == 0) {
      continue;
    }
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      errors_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    const ssize_t size = ::recvfrom(socket_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
    if (size < 0) {
      if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
        errors_.fetch_add(1, std::memory_order_relaxed);
      }
      continue;
    }

    received_.fetch_add(1, std::memory_order_relaxed);
    handler_(buffer.data(), static_cast<std::size_t>(size));
  }
}

}  // namespace asr_sdm::log
