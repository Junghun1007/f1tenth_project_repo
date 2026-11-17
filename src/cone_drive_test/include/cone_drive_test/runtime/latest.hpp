// 스레드 간 "최신 값 하나" 전달 (대기열 없음: 처리가 느리면 오래된 값은 버려진다)
#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>

namespace cone_drive_test {

template<class T>
class Latest {
public:
  void put(std::shared_ptr<const T> v) {
    { std::lock_guard<std::mutex> l(m_); v_ = std::move(v); ++seq_; }
    cv_.notify_all();
  }
  std::shared_ptr<const T> get(std::uint64_t * seq = nullptr) const {
    std::lock_guard<std::mutex> l(m_);
    if (seq) *seq = seq_;
    return v_;
  }
  // seen보다 새 값이 올 때까지 최대 timeout 대기. 없으면 nullptr. seen은 갱신된다
  std::shared_ptr<const T> wait_newer(std::uint64_t & seen, double timeout_sec) const {
    std::unique_lock<std::mutex> l(m_);
    cv_.wait_for(l, std::chrono::duration<double>(timeout_sec), [&] { return seq_ != seen || closed_; });
    if (seq_ == seen || closed_) return nullptr;
    seen = seq_;
    return v_;
  }
  void close() { { std::lock_guard<std::mutex> l(m_); closed_ = true; } cv_.notify_all(); }
  void reopen() { std::lock_guard<std::mutex> l(m_); closed_ = false; }

private:
  mutable std::mutex m_;
  mutable std::condition_variable cv_;
  std::shared_ptr<const T> v_;
  std::uint64_t seq_{0};
  bool closed_{false};
};

}  // namespace cone_drive_test
