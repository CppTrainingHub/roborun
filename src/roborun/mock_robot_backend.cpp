#include "roborun/mock_robot_backend.h"

#include <cmath>
#include <stdexcept>

namespace roborun {

std::string MockRobotBackend::Name() const { return "mock"; }

std::string MockRobotBackend::Version() const { return "roborun-mock-v1"; }

void MockRobotBackend::Connect() { connected_ = true; }

void MockRobotBackend::SetServoEnabled(bool enabled) {
  if (!connected_) {
    throw std::runtime_error("mock backend is not connected");
  }
  servo_enabled_ = enabled;
}

MotionSubmission MockRobotBackend::SubmitMoveJ(const MoveJRequest& request) {
  if (request.speed_percent < 1 || request.speed_percent > 100) {
    throw std::runtime_error("mock backend requires speed_percent in the range 1 to 100");
  }
  if (!std::isfinite(request.joint_speed_radians_per_second) ||
      request.joint_speed_radians_per_second <= 0.0) {
    throw std::runtime_error("mock backend requires a positive finite joint speed");
  }
  if (!connected_ || !servo_enabled_) {
    throw std::runtime_error("mock backend requires an enabled servo");
  }
  last_request_ = request;
  positions_ = request.targets;
  motion_submitted_ = true;
  return MotionSubmission{MotionSubmissionStatus::kAccepted, "", 0};
}

MotionSnapshot MockRobotBackend::PollMotion() {
  if (!connected_ || !motion_submitted_) {
    throw std::runtime_error("mock backend has no submitted motion");
  }
  motion_submitted_ = false;
  return MotionSnapshot{MotionStatus::kSucceeded, positions_, 1, "", std::nullopt};
}

StopSubmission MockRobotBackend::SubmitStop(StopReason) {
  if (!connected_) {
    return {StopSubmissionStatus::kRejected, "mock backend is not connected", std::nullopt};
  }
  motion_submitted_ = false;
  stop_steps_ = 0;
  return {StopSubmissionStatus::kAccepted, "", 0};
}

StopSnapshot MockRobotBackend::PollStop() {
  if (!connected_) {
    return {StopStatus::kFailed, positions_, stop_steps_, "mock backend is not connected",
            std::nullopt};
  }
  ++stop_steps_;
  return {StopStatus::kStopped, positions_, stop_steps_, "", std::nullopt};
}

void MockRobotBackend::Disconnect() {
  connected_ = false;
  servo_enabled_ = false;
  motion_submitted_ = false;
}

}  // namespace roborun
