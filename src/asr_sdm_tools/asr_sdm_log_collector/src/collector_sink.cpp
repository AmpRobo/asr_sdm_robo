#include "asr_sdm_log_collector/collector_sink.hpp"

#include <netdb.h>
#include <spdlog/common.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <string_view>
#include <utility>

namespace asr_sdm::log
{

namespace
{

constexpr std::string_view kTruncationMarker = "...<truncated>\n";

std::size_t clampDatagramLimit(std::size_t requested)
{
  return std::max(requested, kTruncationMarker.size() + 1);
}

}  // namespace

CollectorSink::CollectorSink(const UdpTarget & target, Options options)
: options_{options},
  description_{"udp " + target.host + ":" + std::to_string(target.port)},
  address_{std::make_unique<::sockaddr_storage>()}
{
  options_.max_datagram_bytes = clampDatagramLimit(options_.max_datagram_bytes);
  std::memset(address_.get(), 0, sizeof(::sockaddr_storage));

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;

  const std::string port_text = std::to_string(target.port);
  addrinfo * resolved = nullptr;
  const int status = ::getaddrinfo(target.host.c_str(), port_text.c_str(), &hints, &resolved);
  if (status != 0 || resolved == nullptr) {
    spdlog::throw_spdlog_ex(
      "asr_sdm_log_collector: cannot resolve '" + target.host + "': " + ::gai_strerror(status));
  }

  socket_ =
    ::socket(resolved->ai_family, resolved->ai_socktype | SOCK_NONBLOCK, resolved->ai_protocol);
  if (socket_ < 0) {
    const int socket_errno = errno;
    ::freeaddrinfo(resolved);
    spdlog::throw_spdlog_ex("asr_sdm_log_collector: cannot create UDP socket", socket_errno);
  }

  std::memcpy(address_.get(), resolved->ai_addr, resolved->ai_addrlen);
  address_length_ = resolved->ai_addrlen;
  ::freeaddrinfo(resolved);

  ::setsockopt(
    socket_, SOL_SOCKET, SO_SNDBUF, &options_.send_buffer_bytes,
    sizeof(options_.send_buffer_bytes));
}

CollectorSink::CollectorSink(const UnixTarget & target, Options options)
: options_{options},
  description_{"unix " + target.socket_path},
  address_{std::make_unique<::sockaddr_storage>()}
{
  options_.max_datagram_bytes = clampDatagramLimit(options_.max_datagram_bytes);
  std::memset(address_.get(), 0, sizeof(::sockaddr_storage));

  auto * address = reinterpret_cast<sockaddr_un *>(address_.get());
  if (target.socket_path.empty() || target.socket_path.size() >= sizeof(address->sun_path)) {
    spdlog::throw_spdlog_ex(
      "asr_sdm_log_collector: unusable socket path '" + target.socket_path + "'");
  }
  address->sun_family = AF_UNIX;
  std::memcpy(address->sun_path, target.socket_path.c_str(), target.socket_path.size());
  address_length_ = sizeof(sockaddr_un);

  // Left unbound on purpose: a datagram sender needs no address of its own, and
  // binding one would litter the filesystem with a socket file per process.
  socket_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (socket_ < 0) {
    spdlog::throw_spdlog_ex("asr_sdm_log_collector: cannot create Unix socket", errno);
  }

  ::setsockopt(
    socket_, SOL_SOCKET, SO_SNDBUF, &options_.send_buffer_bytes,
    sizeof(options_.send_buffer_bytes));
}

CollectorSink::~CollectorSink()
{
  if (socket_ >= 0) {
    ::close(socket_);
  }
}

const std::string & CollectorSink::description() const
{
  return description_;
}

uint64_t CollectorSink::sentCount() const
{
  return sent_.load(std::memory_order_relaxed);
}

uint64_t CollectorSink::droppedCount() const
{
  return dropped_.load(std::memory_order_relaxed);
}

void CollectorSink::sink_it_(const spdlog::details::log_msg & msg)
{
  spdlog::memory_buf_t formatted;
  formatter_->format(msg, formatted);

  const char * data = formatted.data();
  std::size_t size = formatted.size();

  std::string truncated;
  if (size > options_.max_datagram_bytes) {
    const std::size_t keep = options_.max_datagram_bytes - kTruncationMarker.size();
    truncated.reserve(options_.max_datagram_bytes);
    truncated.assign(data, keep);
    truncated.append(kTruncationMarker);
    data = truncated.data();
    size = truncated.size();
  }

  const ssize_t written = ::sendto(
    socket_, data, size, MSG_NOSIGNAL, reinterpret_cast<const sockaddr *>(address_.get()),
    static_cast<socklen_t>(address_length_));

  if (written < 0) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
  } else {
    sent_.fetch_add(1, std::memory_order_relaxed);
  }
}

void CollectorSink::flush_()
{
  // Datagrams leave the process on every send; there is nothing buffered here.
}

}  // namespace asr_sdm::log
