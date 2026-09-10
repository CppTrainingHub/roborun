#pragma once

#include "roborun/robot_backend.h"

namespace roborun {

class MockRobotBackend final : public RobotBackend {
 public:
  std::string Name() const override;
  std::string Version() const override;
  void Connect() override;
  void SetServoEnabled(bool enabled) override;
  MotionSubmission SubmitMoveJ(const MoveJRequest& request) override;
  MotionSnapshot PollMotion() override;
  StopSubmission SubmitStop(StopReason reason) override;
  StopSnapshot PollStop() override;
  void Disconnect() override;

  const MoveJRequest& last_request() const { return last_request_; }

 private:
  bool connected_ = false;
  bool servo_enabled_ = false;
  bool motion_submitted_ = false;
  JointPositions positions_{};
  MoveJRequest last_request_{};
  int stop_steps_ = 0;
};

}  // namespace roborun
