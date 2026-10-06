#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace gopro_ros {

/// Minimal fixed-size thread pool. Tasks are started in submission order.
class ThreadPool {
public:
  explicit ThreadPool(std::size_t num_threads) {
    for (std::size_t i = 0; i < num_threads; ++i) {
      workers_.emplace_back([this] { workerLoop(); });
    }
  }

  ~ThreadPool() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    cv_.notify_all();
    for (auto& worker : workers_) worker.join();
  }

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  template <typename F>
  std::future<std::invoke_result_t<F>> submit(F&& task) {
    using Result = std::invoke_result_t<F>;
    auto packaged = std::make_shared<std::packaged_task<Result()>>(std::forward<F>(task));
    std::future<Result> result = packaged->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      tasks_.emplace([packaged] { (*packaged)(); });
    }
    cv_.notify_one();
    return result;
  }

private:
  void workerLoop() {
    while (true) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
        if (stopping_ && tasks_.empty()) return;
        task = std::move(tasks_.front());
        tasks_.pop();
      }
      task();
    }
  }

  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopping_ = false;
};

}  // namespace gopro_ros
