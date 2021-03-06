#ifndef GUIK_PROGRESS_INTERFACE_HPP
#define GUIK_PROGRESS_INTERFACE_HPP

#include <string>

namespace guik {

class ProgressInterface {
 public:
  ProgressInterface() = default;

  virtual ~ProgressInterface() = default;

  virtual void set_title(const std::string &title) {}

  virtual void set_text(const std::string &text) {}

  virtual void set_maximum(int max) {}

  virtual void set_current(int current) {}

  virtual void increment() {}
};

}  // namespace guik

#endif