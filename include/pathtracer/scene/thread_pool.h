#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace pathtracer::scene {

// Persistent worker-thread pool: spawning and joining hardware_concurrency() threads per pass is a measurable share of the frame.
class ThreadPool {
public:
    // One worker per hardware thread, floored at 1. Named rather than a default argument so the spec block reports the number used.
    [[nodiscard]] static unsigned int defaultThreadCount() {
        return std::max(1U, std::thread::hardware_concurrency());
    }

    explicit ThreadPool(unsigned int threadCount = defaultThreadCount());
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    // Blocks until fn(i) has run for every i in [0, count), order unspecified. fn must be safe to call concurrently; not reentrant.
    void parallelFor(int count, const std::function<void(int)>& fn);

    // Worker count, for callers partitioning into per-worker buckets rather than one index per output element.
    [[nodiscard]] unsigned int threadCount() const { return static_cast<unsigned int>(workers_.size()); }

private:
    void workerLoop();

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable dispatchCv_;
    std::condition_variable doneCv_;

    // Bumped by parallelFor to hand off a dispatch; each worker remembers the last epoch it acted on, so a wait cannot miss it.
    std::uint64_t epoch_ = 0;
    bool shuttingDown_ = false;

    const std::function<void(int)>* fn_ = nullptr;
    std::atomic<int> nextIndex_{0};
    int count_ = 0;
    unsigned int workersRemaining_ = 0;
};

}  // namespace pathtracer::scene
