#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace engine::scene {

// Persistent worker-thread pool for parallel rendering. Spawning and joining hardware_concurrency() std::threads on every renderPathTraced call would repeat the OS thread-creation/join cost on every progressive pass under PathTraceDriver; this pool spawns its threads once at construction and parks them (condition_variable wait, no busy-spin) between dispatches instead.
class ThreadPool {
public:
    explicit ThreadPool(unsigned int threadCount = std::max(1U, std::thread::hardware_concurrency()));
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    // Blocks the calling thread until fn(i) has run (on some worker) for every i in [0, count) -- which worker handles which index, and the order, are unspecified; only that all complete before this returns. fn must be safe to call concurrently for different i. Index, not row: the rasterizer dispatches over rows and the path tracer over tiles (path_tracer.cpp), and the pool is indifferent to what the index means. Not reentrant: only one parallelFor call may be in flight at a time (true of every current caller -- PathTraceDriver runs one pass at a time on its single driver thread).
    void parallelFor(int count, const std::function<void(int)>& fn);

    // Worker count, for callers that partition work into per-worker buckets rather than dispatching one index per output element -- the rasterizer's parallel clip/project builds one vector per chunk and needs to size that split to the pool.
    [[nodiscard]] unsigned int threadCount() const { return static_cast<unsigned int>(workers_.size()); }

private:
    void workerLoop();

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable dispatchCv_;
    std::condition_variable doneCv_;

    // Bumped by parallelFor to hand off a new dispatch; each worker remembers the last epoch it acted on so predicate-based waiting can't miss a notification (the epoch, not the notify signal itself, is the source of truth -- immune to the lost-wakeup races a bare notify would risk).
    std::uint64_t epoch_ = 0;
    bool shuttingDown_ = false;

    const std::function<void(int)>* fn_ = nullptr;
    std::atomic<int> nextIndex_{0};
    int count_ = 0;
    unsigned int workersRemaining_ = 0;
};

}  // namespace engine::scene
