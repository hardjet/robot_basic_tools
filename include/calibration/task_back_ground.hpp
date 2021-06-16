#pragma once

#include <atomic>
#include <thread>
#include <string>

#include <boost/any.hpp>

namespace calibration {

/**
 * @brief 后台运行任务
 */
class Task {
 public:
  explicit Task() : is_running_(false) {}

  ~Task() {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  // /**
  //  *
  //  * @tparam T 任务的返回值
  //  * @param task_name 任务名称
  //  * @param task 任务函数
  //  */
  // template <typename T, typename... ArgTypes>
  // bool do_task(const std::string &task_name, std::function<T(ArgTypes...)> &task) {
  //   if (is_new_) {
  //     is_new_ = false;
  //     create(task_name, task);
  //   }
  //
  //   return is_terminated(task_name);
  // }
  //
  // /**
  //  *
  //  * @tparam T 任务的返回值
  //  * @param task_name 任务名称
  //  * @param task 任务函数
  //  */
  // template <typename T, typename... ArgTypes>
  // void create(const std::string &task_name, std::function<T(ArgTypes...)> &task) {
  //   task_name_ = task_name;
  //
  //   result_.clear();
  //   is_running_ = true;
  //
  //   thread_ = std::thread([this, task]() {
  //     result_ = task();
  //     is_running_ = false;
  //   });
  // }

  /**
   * @brief 后台运行一个任务
   * @tparam Function 任务函数类型
   * @tparam Args 任务函数参数类型
   * @param task_name 任务名称
   * @param task 任务函数
   * @param args 任务函数参数
   * @return 任务是否结束
   */
  template <typename Function, typename... Args>
  bool do_task(const std::string &task_name, Function &&task, Args &&...args) {
    if (is_new_) {
      is_new_ = false;
      create(task_name, task);
    }

    return is_terminated(task_name);
  }

  /**
   * @brief 创建一个后台执行任务，并运行
   * @tparam Function 任务函数类型
   * @tparam Args 任务函数参数类型
   * @param task_name 任务名称
   * @param task 任务函数
   * @param args 任务函数参数
   */
  template <typename Function, typename... Args>
  void create(const std::string &task_name, Function &&task, Args &&...args) {
    task_name_ = task_name;

    result_.clear();
    is_running_ = true;

    thread_ = std::thread([this, task, args...]() {
      result_ = task(args...);
      is_running_ = false;
    });
  }

  /**
   *
   * @tparam T 任务结果类型
   * @return 任务结果
   */
  template <typename T>
  T result() {
    T ret = boost::any_cast<T>(result_);
    result_.clear();
    return ret;
  }

  /**
   * @brief 指定名称的任务是否结束
   * @param task_name 指定任务
   * @return 任务是否结束
   */
  bool is_terminated(const std::string &task_name) {
    if (task_name != task_name_) {
      return false;
    }

    bool terminated = false;
    // 如果任务结束
    if (!is_running_) {
      thread_.join();
      // 清除任务名称
      task_name_ = "";
      is_new_ = true;
      terminated = true;
    }

    return terminated;
  }

 private:
  // 是否是新任务
  bool is_new_{true};
  // 当前任务名称
  std::string task_name_;
  // 是否在运行
  std::atomic_bool is_running_;
  // 线程
  std::thread thread_;
  // 任务结果
  boost::any result_;
};

}  // namespace calibration
