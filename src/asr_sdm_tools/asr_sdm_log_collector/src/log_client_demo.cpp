/// Exercises the log client end to end: run the collector, then run this, and the
/// records should appear in the collector's per-node file.
///
///   ros2 run asr_sdm_log_collector asr_sdm_log_client_demo [node_name] [rounds]

#include "asr_sdm_log_collector/log_client.hpp"

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

int main(int argc, char * argv[])
{
  const std::string node_name = argc > 1 ? argv[1] : "log_client_demo";
  const int rounds = argc > 2 ? std::atoi(argv[2]) : 5;

  auto config = asr_sdm::log::configFromEnv(node_name);
  config.level = spdlog::level::trace;
  asr_sdm::log::initialize(config);

  for (int round = 0; round < rounds; ++round) {
    SPDLOG_TRACE("tick {}", round);
    SPDLOG_DEBUG("uart queue depth {}", round * 2);
    SPDLOG_INFO("joint {} at {:.3f} rad", round, 0.125 * round);
    SPDLOG_WARN("screw current {:.1f} A above nominal", 3.2 + round);
    SPDLOG_ERROR("CAN frame {} timed out", round);
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
  }

  SPDLOG_INFO("finished, {} record(s) dropped", asr_sdm::log::droppedRecordCount());
  asr_sdm::log::shutdown();
  return 0;
}
