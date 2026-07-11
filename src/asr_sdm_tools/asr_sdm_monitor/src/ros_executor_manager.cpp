#include "asr_sdm_monitor/ros_executor_manager.hpp"

#include <array>
#include <cstddef>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>

namespace asr_sdm_monitor
{

namespace
{
constexpr std::size_t roleIndex(RosExecutorRole role)
{
    return static_cast<std::size_t>(role);
}
}  // namespace

class RosExecutorManager::Impl
{
public:
    struct Worker
    {
        std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
            std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        std::vector<rclcpp::Node::SharedPtr> nodes;
        std::thread thread;
    };

    std::array<Worker, 3> workers;
    mutable std::mutex lifecycle_mutex;
    bool started = false;
};

RosExecutorManager::RosExecutorManager()
    : impl_(std::make_unique<Impl>())
{
}

RosExecutorManager::~RosExecutorManager()
{
    stop();
}

bool RosExecutorManager::addNode(
    RosExecutorRole role,
    const rclcpp::Node::SharedPtr &node)
{
    if (!node) {
        return false;
    }

    std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
    if (impl_->started) {
        return false;
    }

    auto &worker = impl_->workers[roleIndex(role)];
    worker.executor->add_node(node);
    worker.nodes.push_back(node);
    return true;
}

void RosExecutorManager::start()
{
    std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
    if (impl_->started) {
        return;
    }

    impl_->started = true;
    try {
        for (auto &worker : impl_->workers) {
            worker.thread = std::thread([executor = worker.executor.get()]()
            {
                executor->spin();
            });
        }
    } catch (...) {
        for (auto &worker : impl_->workers) {
            worker.executor->cancel();
        }
        for (auto &worker : impl_->workers) {
            if (worker.thread.joinable()) {
                worker.thread.join();
            }
        }
        impl_->started = false;
        throw;
    }
}

void RosExecutorManager::stop() noexcept
{
    std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
    if (!impl_->started) {
        return;
    }

    for (auto &worker : impl_->workers) {
        worker.executor->cancel();
    }
    for (auto &worker : impl_->workers) {
        if (worker.thread.joinable()) {
            worker.thread.join();
        }
    }
    impl_->started = false;
}

bool RosExecutorManager::isStarted() const noexcept
{
    std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
    return impl_->started;
}

}  // namespace asr_sdm_monitor
