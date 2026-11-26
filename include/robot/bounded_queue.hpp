/*
    “满队丢最旧帧”的实时队列
    通俗直觉：想象一条只有 1 个格子的传送带。摄像头每秒送 30 张图（生产快），RK3568 的 NPU 每秒只能算 17 张（消费慢）。
    如果让队列无限变长：内存迟早爆掉，而且 AI 看到的画面越来越延迟（比如看到的是 5 秒前的障碍物）。
    如果让摄像头原地等 NPU 算完：整个系统被拖慢到 17 fps，连另一条 30 fps 的录像/推流线程也会一起卡顿。
    本项目的做法：格子满了就把格子里最旧的那张踢掉，放进最新的一张（push_latest）。
*/

#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace robot {

// 实时视频不能无限积压。满队列时丢弃最旧帧，优先保证时延而不是逐帧完整性。
template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {}

    bool push_latest(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) return false;
        if (queue_.size() == capacity_) {
            queue_.pop_front();
            ++dropped_;
        }
        queue_.push_back(std::move(value));
        not_empty_.notify_one();
        return true;
    }

    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return false;
        value = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        not_empty_.notify_all();
    }

    std::size_t dropped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

private:
    const std::size_t capacity_;
    std::deque<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    bool closed_{false};
    std::size_t dropped_{0};
};

}  // namespace robot
