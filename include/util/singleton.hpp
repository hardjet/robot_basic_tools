// reference to https://my.oschina.net/u/3102542/blog/1563222

#pragma once

namespace util {

#include <mutex>
#include <memory>

template <typename T>
class Singleton {
 public:
  //获取全局单例对象
  template <typename... Args>
  static std::shared_ptr<T> instance(Args&&... args) {
    static std::shared_ptr<T> singleton_inst_;
    static std::mutex mtx_;
    if (!singleton_inst_) {
      std::lock_guard<std::mutex> gLock(mtx_);
      if (nullptr == singleton_inst_) {
        singleton_inst_ = std::make_shared<T>(std::forward<Args>(args)...);
      }
    }
    return singleton_inst_;
  }

  /// 禁用拷贝构造
  Singleton(const Singleton&) = delete;

  /// 禁用赋值操作
  Singleton& operator=(const Singleton&) = delete;

 private:
  explicit Singleton() = default;
};

// template <typename T>
// std::shared_ptr<T> Singleton<T>::singleton_inst_{nullptr};
//
// template <typename T>
// std::mutex Singleton<T>::mtx_;

}  // namespace util
