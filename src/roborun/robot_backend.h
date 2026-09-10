#pragma once

#include <optional>
#include <string>

#include "roborun/types.h"

namespace roborun {

enum class MotionSubmissionStatus {
  kAccepted,
  kRejected,
};

enum class MotionStatus {
  kRunning,
  kSucceeded,
  kFailed,
};

struct MotionSubmission {
  MotionSubmissionStatus status = MotionSubmissionStatus::kRejected;
  std::string message;
  std::optional<BusinessTime> poll_after_ms;
};

struct MotionSnapshot {
  MotionStatus status = MotionStatus::kFailed;
  JointPositions positions{};
  int backend_steps = 0;
  std::string message;
  std::optional<BusinessTime> poll_after_ms;
};

enum class StopSubmissionStatus {
  kAccepted,
  kRejected,
};

struct StopSubmission {
  StopSubmissionStatus status = StopSubmissionStatus::kRejected;
  std::string message;
  std::optional<BusinessTime> poll_after_ms;
};

struct StopSnapshot {
  StopStatus status = StopStatus::kFailed;
  JointPositions positions{};
  int backend_steps = 0;
  std::string message;
  std::optional<BusinessTime> poll_after_ms;
};

class RobotBackend {
 public:
  virtual ~RobotBackend() = default;

  virtual std::string Name() const = 0;
  virtual std::string Version() const = 0;
  // Reset per-program bookkeeping only; must not perform device operations.
  virtual void BeginProgram() noexcept {}
  virtual void Connect() = 0;
  virtual void SetServoEnabled(bool enabled) = 0;
  virtual MotionSubmission SubmitMoveJ(const MoveJRequest& request) = 0;
  // Resume the same Program command after a confirmed pause stop.
  virtual MotionSubmission ResumeMoveJ(const MoveJRequest& request) { return SubmitMoveJ(request); }
  virtual MotionSnapshot PollMotion() = 0;
  virtual StopSubmission SubmitStop(StopReason reason) = 0;
  virtual StopSnapshot PollStop() = 0;
  virtual void Disconnect() = 0;
};

}  // namespace roborun
