#ifndef ASR_SDM_LOG_COLLECTOR__DATAGRAM_RECEIVER_HPP_
#define ASR_SDM_LOG_COLLECTOR__DATAGRAM_RECEIVER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace asr_sdm::log
{

/// Owns one listening socket and pushes every datagram it receives into a
/// handler on a dedicated thread.
///
/// The collector runs one of these per transport it accepts, so a Unix socket and
/// a UDP port can feed the same writer at the same time.
class DatagramReceiver
{
public:
  using DatagramHandler = std::function<void(const char *, std::size_t)>;

  struct UdpEndpoint
  {
    std::string bind_address{"0.0.0.0"};
    uint16_t port{9110};
  };

  struct UnixEndpoint
  {
    std::string socket_path;
    /// Senders commonly run under a different account than the collector, so the
    /// socket is world-writable unless told otherwise.
    unsigned int permissions{0666};
  };

  struct Options
  {
    std::size_t max_datagram_bytes{65536};
    /// A generous kernel buffer absorbs bursts while the writer rotates files.
    int receive_buffer_bytes{4 * 1024 * 1024};
  };

  /// Both throw std::runtime_error when the socket cannot be opened or bound, so
  /// a port clash or an unwritable socket directory surfaces at startup instead
  /// of as silently missing logs.
  DatagramReceiver(const UdpEndpoint & endpoint, Options options, DatagramHandler handler);
  DatagramReceiver(const UnixEndpoint & endpoint, Options options, DatagramHandler handler);
  ~DatagramReceiver();

  DatagramReceiver(const DatagramReceiver &) = delete;
  DatagramReceiver & operator=(const DatagramReceiver &) = delete;

  void start();
  void stop();

  /// Where it is listening, for start-up messages.
  const std::string & description() const;
  uint64_t receivedCount() const;
  uint64_t errorCount() const;

private:
  void applyCommonSocketOptions();
  void receiveLoop();

  Options options_;
  DatagramHandler handler_;
  std::string description_;
  /// Set only for Unix endpoints, whose socket file must be removed on teardown.
  std::string owned_socket_path_;
  int socket_{-1};
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> received_{0};
  std::atomic<uint64_t> errors_{0};
};

}  // namespace asr_sdm::log

#endif  // ASR_SDM_LOG_COLLECTOR__DATAGRAM_RECEIVER_HPP_
