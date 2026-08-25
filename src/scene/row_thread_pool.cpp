#include "engine/scene/row_thread_pool.h"

namespace engine::scene {

RowThreadPool::RowThreadPool(unsigned int threadCount) {
    workers_.reserve(threadCount);
    for (unsigned int i = 0; i < threadCount; ++i) {
        workers_.emplace_back([this] { workerLoop(); });
    }
}

RowThreadPool::~RowThreadPool() {
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

void RowThreadPool::parallelForRows(int rowCount, const std::function<void(int)>& rowFn) {
    if (rowCount <= 0) {
        return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    rowFn_ = &rowFn;
    rowCount_ = rowCount;
    nextRow_.store(0, std::memory_order_relaxed);
    workersRemaining_ = static_cast<unsigned int>(workers_.size());
    ++epoch_;
    lock.unlock();
    dispatchCv_.notify_all();

    lock.lock();
    doneCv_.wait(lock, [this] { return workersRemaining_ == 0; });
    rowFn_ = nullptr;
}

void RowThreadPool::workerLoop() {
    std::uint64_t lastEpoch = 0;
    while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        dispatchCv_.wait(lock, [this, lastEpoch] { return epoch_ != lastEpoch; });
        lastEpoch = epoch_;
        if (shuttingDown_) {
            return;
        }
        const std::function<void(int)>* rowFn = rowFn_;
        const int rowCount = rowCount_;
        lock.unlock();

        int y = 0;
        while ((y = nextRow_.fetch_add(1, std::memory_order_relaxed)) < rowCount) {
            (*rowFn)(y);
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

}  // namespace engine::scene
