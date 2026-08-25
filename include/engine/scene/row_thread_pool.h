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

// Persistent worker-thread pool for parallel row-based rendering. renderPathTraced used to spawn and
// join hardware_concurrency() std::threads on every single call; under PathTraceDriver that happens
// once per progressive pass, so the OS thread-creation/join cost repeated every pass. This pool
// spawns its threads once at construction and parks them (condition_variable wait, no busy-spin)
// between dispatches instead.
class RowThreadPool {
public:
    explicit RowThreadPool(unsigned int threadCount = std::max(1U, std::thread::hardware_concurrency()));
    ~RowThreadPool();

    RowThreadPool(const RowThreadPool&) = delete;
    RowThreadPool& operator=(const RowThreadPool&) = delete;
    RowThreadPool(RowThreadPool&&) = delete;
    RowThreadPool& operator=(RowThreadPool&&) = delete;

    // Blocks the calling thread until rowFn(y) has run (on some worker) for every y in
    // [0, rowCount) -- which worker handles which row, and the order, are unspecified; only that all
    // complete before this returns. rowFn must be safe to call concurrently for different y. Not
    // reentrant: only one parallelForRows call may be in flight at a time (true of every current
    // caller -- PathTraceDriver runs one pass at a time on its single driver thread).
    void parallelForRows(int rowCount, const std::function<void(int)>& rowFn);

private:
    void workerLoop();

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable dispatchCv_;
    std::condition_variable doneCv_;

    // Bumped by parallelForRows to hand off a new dispatch; each worker remembers the last epoch it
    // acted on so predicate-based waiting can't miss a notification (the epoch, not the notify signal
    // itself, is the source of truth -- immune to the lost-wakeup races a bare notify would risk).
    std::uint64_t epoch_ = 0;
    bool shuttingDown_ = false;

    const std::function<void(int)>* rowFn_ = nullptr;
    std::atomic<int> nextRow_{0};
    int rowCount_ = 0;
    unsigned int workersRemaining_ = 0;
};

}  // namespace engine::scene
