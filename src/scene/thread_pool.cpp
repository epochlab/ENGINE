#include "pathtracer/scene/thread_pool.h"

#include <algorithm>
#include <utility>

namespace pathtracer::scene {

ThreadPool::ThreadPool(unsigned int threadCount) {
    const unsigned int count = std::max(1U, threadCount);
    workers_.reserve(count);
    for (unsigned int i = 0; i < count; ++i) {
        workers_.emplace_back([this] { workerLoop(); });
    }
}

ThreadPool::~ThreadPool() {
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        shuttingDown_ = true;
        ++epoch_;
    }
    dispatchCv_.notify_all();
    for (std::thread& worker : workers_) {
        worker.join();
    }
}

void ThreadPool::parallelFor(int count, const std::function<void(int)>& fn) {
    if (count <= 0) {
        return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    fn_ = &fn;
    count_ = count;
    nextIndex_.store(0, std::memory_order_relaxed);
    workersRemaining_ = static_cast<unsigned int>(workers_.size());
    ++epoch_;
    lock.unlock();
    dispatchCv_.notify_all();

    lock.lock();
    doneCv_.wait(lock, [this] { return workersRemaining_ == 0; });
    fn_ = nullptr;
    // Rethrown on the caller's thread, after every worker is accounted for: the pass is invalid, and swallowing it would publish it.
    if (firstException_) {
        std::rethrow_exception(std::exchange(firstException_, nullptr));
    }
}

void ThreadPool::workerLoop() {
    std::uint64_t lastEpoch = 0;
    while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        dispatchCv_.wait(lock, [this, lastEpoch] { return epoch_ != lastEpoch; });
        lastEpoch = epoch_;
        if (shuttingDown_) {
            return;
        }
        const std::function<void(int)>* fn = fn_;
        const int count = count_;
        lock.unlock();

        // A throwing task must still reach the decrement below, which parallelFor waits on -- an escape would hang the caller forever.
        try {
            int index = 0;
            while ((index = nextIndex_.fetch_add(1, std::memory_order_relaxed)) < count) {
                (*fn)(index);
            }
        } catch (...) {
            const std::lock_guard<std::mutex> guard(mutex_);
            if (!firstException_) {
                firstException_ = std::current_exception();
            }
        }

        lock.lock();
        --workersRemaining_;
        const bool isLast = workersRemaining_ == 0;
        lock.unlock();
        if (isLast) {
            doneCv_.notify_one();
        }
    }
}

}  // namespace pathtracer::scene
