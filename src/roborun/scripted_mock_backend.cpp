#include "roborun/scripted_mock_backend.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace roborun {

ScriptedMockBackend::ScriptedMockBackend(MockScenario scenario) : scenario_(std::move(scenario)) {}

std::string ScriptedMockBackend::Name() const { return "scripted-mock"; }

std::string ScriptedMockBackend::Version() const { return "roborun-scripted-mock-v1"; }

void ScriptedMockBackend::BeginProgram() noexcept {
  movej_occurrence_ = 0;
  stop_requests_ = 0;
  motion_submitted_ = false;
  stop_submitted_ = false;
}

void ScriptedMockBackend::Connect() {
  call_log_.push_back("connect");
  connected_ = true;
}

void ScriptedMockBackend::SetServoEnabled(bool enabled) {
  call_log_.push_back(enabled ? "servo_on" : "servo_off");
  if (!connected_) {
    throw std::runtime_error("scripted mock backend is not connected");
  }
  servo_enabled_ = enabled;
}

MotionSubmission ScriptedMockBackend::SubmitMoveJ(const MoveJRequest& request) {
  return SubmitMotion(request, false);
}

MotionSubmission ScriptedMockBackend::ResumeMoveJ(const MoveJRequest& request) {
  return SubmitMotion(request, true);
}

MotionSubmission ScriptedMockBackend::SubmitMotion(const MoveJRequest& request, bool resume) {
  call_log_.push_back("submit_movej");
  if (!connected_ || !servo_enabled_) {
    throw std::runtime_error("scripted mock backend requires an enabled servo");
  }
  if (request.speed_percent < 1 || request.speed_percent > 100 ||
      !std::isfinite(request.joint_speed_radians_per_second) ||
      request.joint_speed_radians_per_second <= 0.0) {
    throw std::runtime_error("scripted mock backend received invalid motion parameters");
  }
  last_request_ = request;
  if (!resume) {
    ++movej_occurrence_;
  }
  motion_submitted_ = true;
  motion_frozen_ = false;
  const auto plan = scenario_.motion_plans.find(movej_occurrence_);
  if (plan == scenario_.motion_plans.end() ||
      plan->second.outcome == MotionPlanOutcome::kImmediateSuccess) {
    return MotionSubmission{MotionSubmissionStatus::kAccepted, "", 0};
  }
  if (plan->second.outcome == MotionPlanOutcome::kRejected) {
    motion_submitted_ = false;
    return MotionSubmission{MotionSubmissionStatus::kRejected, "motion target was rejected", 0};
  }
  if (plan->second.outcome == MotionPlanOutcome::kFrozen) {
    motion_frozen_ = true;
    return MotionSubmission{MotionSubmissionStatus::kAccepted, "", 0};
  }
  return MotionSubmission{MotionSubmissionStatus::kAccepted, "", plan->second.delay_ms};
}

MotionSnapshot ScriptedMockBackend::PollMotion() {
  call_log_.push_back("poll_motion");
  if (!connected_ || !motion_submitted_) {
    throw std::runtime_error("scripted mock backend has no submitted motion");
  }
  if (motion_frozen_) {
    return MotionSnapshot{MotionStatus::kRunning, positions_, 0, "", std::nullopt};
  }
  positions_ = last_request_.targets;
  motion_submitted_ = false;
  return MotionSnapshot{MotionStatus::kSucceeded, positions_, 1, "", std::nullopt};
}

StopSubmission ScriptedMockBackend::SubmitStop(StopReason reason) {
  call_log_.push_back("submit_stop:" + ToString(reason));
  ++stop_requests_;
  if (!connected_) {
    return {StopSubmissionStatus::kRejected, "scripted mock backend is not connected",
            std::nullopt};
  }
  stop_submitted_ = true;
  stop_failed_ = false;
  stop_frozen_ = false;
  stop_delay_ms_ = 0;
  stop_polls_ = 0;
  const auto plan = scenario_.stop_plans.find(static_cast<std::size_t>(stop_requests_));
  if (plan == scenario_.stop_plans.end() ||
      plan->second.outcome == StopPlanOutcome::kImmediateConfirmation) {
    return {StopSubmissionStatus::kAccepted, "", 0};
  }
  if (plan->second.outcome == StopPlanOutcome::kRejected) {
    stop_submitted_ = false;
    return {StopSubmissionStatus::kRejected, "stop request was rejected", std::nullopt};
  }
  if (plan->second.outcome == StopPlanOutcome::kNeverConfirmed) {
    stop_frozen_ = true;
    return {StopSubmissionStatus::kAccepted, "", 0};
  }
  if (plan->second.outcome == StopPlanOutcome::kFailed) {
    stop_failed_ = true;
    return {StopSubmissionStatus::kAccepted, "", 0};
  }
  stop_delay_ms_ = plan->second.delay_ms;
  return {StopSubmissionStatus::kAccepted, "", 0};
}

StopSnapshot ScriptedMockBackend::PollStop() {
  call_log_.push_back("poll_stop");
  if (!connected_ || !stop_submitted_) {
    return {StopStatus::kFailed, positions_, stop_polls_, "stop was not submitted", std::nullopt};
  }
  ++stop_polls_;
  if (stop_failed_) {
    return {StopStatus::kFailed, positions_, stop_polls_, "stop polling failed", std::nullopt};
  }
  if (stop_frozen_) {
    return {StopStatus::kStopping, positions_, stop_polls_, "", 10};
  }
  if (stop_delay_ms_ > 0 && stop_polls_ == 1) {
    return {StopStatus::kStopping, positions_, stop_polls_, "", stop_delay_ms_};
  }
  motion_submitted_ = false;
  stop_submitted_ = false;
  return {StopStatus::kStopped, positions_, stop_polls_, "", std::nullopt};
}

void ScriptedMockBackend::Disconnect() {
  call_log_.push_back("disconnect");
  connected_ = false;
  servo_enabled_ = false;
  motion_submitted_ = false;
}

}  // namespace roborun
