#include "calibration/calibration_state.hpp"
#include "calibration/camera_calib.hpp"

namespace calibration {

std::string StateIdle::name() { return "STATE_IDLE"; }
std::string StateStart::name() { return "STATE_START"; }
std::string StateGetPose::name() { return "STATE_GET_POSE"; }
std::string StateCheckSteady::name() { return "STATE_CHECK_STEADY"; }
std::string StateStartCalib::name() { return "STATE_START_CALIB"; }
std::string StateInCalib::name() {return "STATE_IN_CALIB"; }

int StateIdle::id() { return 0; }
int StateStart::id() { return 1; }
int StateGetPose::id() { return 2; }
int StateCheckSteady::id() { return 3; }
int StateStartCalib::id() { return 4; }
int StateInCalib::id() { return 5; }

void StateIdle::calibration() {
  calib_context_->change_current_state(calib_context_->next_state());
}

void StateStart::calibration() {
  if (calib_context_->instrument_available()) {
    calib_context_->change_current_state(std::make_shared<StateGetPose>());
  } else {
    calib_context_->change_current_state(std::make_shared<StateIdle>());
  }
}

void StateGetPose::calibration() {
  if (calib_context_->pose_valid()) {
    calib_context_->change_current_state(std::make_shared<StateCheckSteady>());
  } else {
    calib_context_->change_current_state(std::make_shared<StateIdle>());
  }
}

void StateCheckSteady::calibration() {
  calib_context_->check_steady();
  calib_context_->change_current_state(std::make_shared<StateIdle>());
}

void StateStartCalib::calibration() {
  calib_context_->change_current_state(std::make_shared<StateInCalib>());
}

void StateInCalib::calibration() {
  if (calib_context_->do_calib()) {
    calib_context_->change_current_state(std::make_shared<StateIdle>());
  } else {
    std::cout << "calibration failed!!!" << std::endl;
  }
}

}  // namespace calibration