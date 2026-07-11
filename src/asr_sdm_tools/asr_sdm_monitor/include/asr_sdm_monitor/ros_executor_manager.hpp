#pragma once

#include <memory>

#include <rclcpp/node.hpp>

namespace asr_sdm_monitor
{

enum class RosExecutorRole
{
    Plot,
    Video,
    Hardware
};

// Owns the ROS worker executors and their threads. Keeping lifecycle management
// outside RosUiBridge makes start/stop ordering explicit and idempotent.
class RosExecutorManager final
{
public:
    RosExecutorManager();
    ~RosExecutorManager();

    RosExecutorManager(const RosExecutorManager &) = delete;
    RosExecutorManager &operator=(const RosExecutorManager &) = delete;

    bool addNode(RosExecutorRole role, const rclcpp::Node::SharedPtr &node);
    void start();
    void stop() noexcept;
    bool isStarted() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace asr_sdm_monitor
