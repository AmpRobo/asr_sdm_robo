#ifndef ASR_SDM_LOG_COLLECTOR__COLLECTOR_SINK_HPP_
#define ASR_SDM_LOG_COLLECTOR__COLLECTOR_SINK_HPP_

#include <spdlog/sinks/base_sink.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

// Declared here rather than pulling <sys/socket.h> into a public header.
struct sockaddr_storage;

namespace asr_sdm::log
{

/// Ships formatted records to `asr_sdm_log_collector` as datagrams.
///
/// Two targets are supported. A Unix socket is the better choice when the
/// collector runs on the same host: delivery is not silently lossy, it costs
/// less than the network stack, and access is governed by file permissions. UDP
/// is there for senders on another machine.
///
/// spdlog bundles a udp_sink, but it resolves the target with inet_aton, so host
/// names are rejected, and it throws on every failed send, which turns an absent
/// collector into a flood of error-handler output. This sink resolves through
/// getaddrinfo, keeps the socket non-blocking so a full send buffer can never
/// stall a control loop, and counts drops instead of throwing.
class CollectorSink : public spdlog::sinks::base_sink<std::mutex>
{
public:
  struct UdpTarget
  {
    std::string host{"127.0.0.1"};
    uint16_t port{9110};
  };

  struct UnixTarget
  {
    std::string socket_path;
  };

  struct Options
  {
    int send_buffer_bytes{1 << 20};
    /// Records longer than this are truncated rather than dropped. Kept well
    /// below the 65507-byte IPv4 payload ceiling to avoid fragmentation.
    std::size_t max_datagram_bytes{60000};
  };

  /// Both throw spdlog::spdlog_ex when the target cannot be resolved or the
  /// socket cannot be opened.
  CollectorSink(const UdpTarget & target, Options options);
  CollectorSink(const UnixTarget & target, Options options);
  ~CollectorSink() override;

  CollectorSink(const CollectorSink &) = delete;
  CollectorSink & operator=(const CollectorSink &) = delete;

  /// Where records are being sent, for start-up messages.
  const std::string & description() const;

  uint64_t sentCount() const;
  uint64_t droppedCount() const;

protected:
  void sink_it_(const spdlog::details::log_msg & msg) override;
  void flush_() override;

private:
  Options options_;
  std::string description_;
  int socket_{-1};
  // Held by pointer so the socket headers stay out of this public header.
  std::unique_ptr<::sockaddr_storage> address_;
  std::size_t address_length_{0};
  std::atomic<uint64_t> sent_{0};
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace asr_sdm::log

#endif  // ASR_SDM_LOG_COLLECTOR__COLLECTOR_SINK_HPP_
