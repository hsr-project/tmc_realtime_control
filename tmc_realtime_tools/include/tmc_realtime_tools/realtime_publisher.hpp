/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef TMC_REALTIME_TOOLS_REALTIME_PUBLISHER_HPP_
#define TMC_REALTIME_TOOLS_REALTIME_PUBLISHER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <immintrin.h>

#include <rclcpp/rclcpp.hpp>

namespace tmc_realtime_tools {

template<class MessageT, size_t QueueSize = 64>
class RealtimePublisher {
 public:
  static_assert((QueueSize & (QueueSize - 1)) == 0, "QueueSize must be power-of-two");

  using Ptr = std::shared_ptr<RealtimePublisher<MessageT, QueueSize>>;

  using PublisherPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;
  using InitFunc = std::function<void(MessageT&)>;

  explicit RealtimePublisher(PublisherPtr pub, InitFunc init_func = nullptr, uint32_t spin_limit = 100)
      : publisher_(pub), head_(0), tail_(0), running_(true), spin_limit_(spin_limit) {
    buffers_.resize(QueueSize);
    if (init_func) {
      for (auto& msg : buffers_) {
        init_func(msg);
      }
    }

    thread_ = std::thread(&RealtimePublisher::PublishLoop, this);
  }

  ~RealtimePublisher() {
    running_.store(false);

    head_.fetch_add(1, std::memory_order_relaxed);
    head_.notify_all();

    if (thread_.joinable()) {
      thread_.join();
    }
  }

  // Match the public function name to realtime_tools::RealtimePublisher
  MessageT* trylock() {
    auto head = head_.load(std::memory_order_relaxed);
    auto next = (head + 1) & mask_;

    if (next == tail_.load(std::memory_order_acquire)) {
      return nullptr;
    }
    return &buffers_[head];
  }

  void unlockAndPublish() {
    auto head = head_.load(std::memory_order_relaxed);
    auto next = (head + 1) & mask_;

    head_.store(next, std::memory_order_release);
    head_.notify_one();
  }

 private:
  void PublishLoop() {
    uint32_t spin = 0;

    while (running_.load()) {
      auto tail = tail_.load(std::memory_order_relaxed);
      auto head = head_.load(std::memory_order_acquire);

      if (tail == head) {
        if (spin < spin_limit_) {
          spin++;
          _mm_pause();
        } else {
          while (tail == head && running_.load()) {
            head_.wait(head);
            head = head_.load(std::memory_order_acquire);
          }
        }
        if (!running_.load()) {
          return;
        }
        continue;
      }

      spin = 0;

      MessageT& msg = buffers_[tail];
      tail_.store((tail + 1) & mask_, std::memory_order_release);

      publisher_->publish(msg);
    }
  }

 private:
  PublisherPtr publisher_;

  static constexpr uint32_t mask_ = QueueSize - 1;

  std::vector<MessageT> buffers_;

  std::atomic<uint32_t> head_;
  std::atomic<uint32_t> tail_;

  std::atomic<bool> running_;
  uint32_t spin_limit_;

  std::thread thread_;
};

}  // namespace tmc_realtime_tools
#endif  // TMC_REALTIME_TOOLS_REALTIME_PUBLISHER_HPP_
