#pragma once

#include <utility>
#include <iostream>
#include <vector>

namespace calibration {

// typedef void(*func_ptr)();
class BaseCalib;

class CalibrationState {
 public:
  CalibrationState() : calib_context_(nullptr) {}
  virtual ~CalibrationState() = default;

  virtual int id() = 0;
  virtual std::string name() = 0;
  virtual void calibration() = 0;
  void set_context(BaseCalib* context) { calib_context_ = context; }

 public:
  // std::vector<func_ptr> tasks_;
  BaseCalib* calib_context_;
};

class StateIdle : public CalibrationState {
 public:
  int id() override;
  std::string name() override;
  void calibration() override;
};

class StateStart : public CalibrationState {
 public:
  int id() override;
  std::string name() override;
  void calibration() override;
};

class StateGetPose : public CalibrationState {
 public:
  int id() override;
  std::string name() override;
  void calibration() override;
};

class StateCheckSteady : public CalibrationState {
 public:
  int id() override;
  std::string name() override;
  void calibration() override;
};

class StateStartCalib : public CalibrationState {
 public:
  int id() override;
  std::string name() override;
  void calibration() override;
};

class StateInCalib : public CalibrationState {
 public:
  int id() override;
  std::string name() override;
  void calibration() override;
};

}  // namespace calibration