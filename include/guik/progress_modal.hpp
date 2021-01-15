#ifndef GUIK_PROGRESS_MODAL_HPP
#define GUIK_PROGRESS_MODAL_HPP

#include <mutex>
#include <atomic>
#include <thread>
#include <string>
#include <iostream>

#include <imgui.h>
#include <boost/any.hpp>

#include <guik/progress_interface.hpp>
#include <utility>

namespace guik {

class ProgressModal : public ProgressInterface {
 public:
  explicit ProgressModal(std::string modal_name)
      : modal_name_(std::move(modal_name)), running_(false), max_(0), current_(0) {}

  ~ProgressModal() override {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void set_title(const std::string &title) override {
    std::lock_guard<std::mutex> lock(mutex);
    this->title_ = title;
  }

  void set_text(const std::string &text) override {
    std::lock_guard<std::mutex> lock(mutex);
    this->text_ = text;
  }

  void set_maximum(int max) override { this->max_ = max; }

  void set_current(int current) override { this->current_ = current; }

  void increment() override { current_++; }

  template <typename T>
  void open(const std::string &task_name, const std::function<T(ProgressInterface &progress)> &task) {
    this->task_name_ = task_name;
    this->title_.clear();
    this->text_.clear();

    ImGui::OpenPopup(modal_name_.c_str());
    result_.clear();
    running_ = true;
    current_ = 0;

    thread_ = std::thread([this, task]() {
      result_ = task(*this);
      running_ = false;
    });
  }

  template <typename T>
  T result() {
    T ret = boost::any_cast<T>(result_);
    result_.clear();
    return ret;
  }

  bool run(const std::string &task_name) {
    if (task_name != this->task_name_) {
      return false;
    }

    bool terminated = false;
    if (ImGui::BeginPopupModal(modal_name_.c_str(), nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (!title_.empty()) {
          ImGui::Text("%s", title_.c_str());
        }

        ImGui::Text("%c %s", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3], text_.c_str());
      }

      float fraction = current_ / static_cast<float>(max_);
      ImGui::ProgressBar(fraction, ImVec2(256, 16));

      if (!running_) {
        thread_.join();
        ImGui::CloseCurrentPopup();
        terminated = true;
      }
      ImGui::EndPopup();
    }

    return terminated;
  }

 private:
  std::mutex mutex;
  std::string modal_name_;
  std::string title_;
  std::string text_;

  std::atomic_bool running_;
  std::atomic_int max_;
  std::atomic_int current_;

  std::string task_name_;

  std::thread thread_;
  boost::any result_;
};

}  // namespace guik

#endif