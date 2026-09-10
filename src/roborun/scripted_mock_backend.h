#pragma once

#include <string>
#include <vector>

#include "roborun/mock_robot_backend.h"
#include "roborun/scenario.h"

namespace roborun {

class ScriptedMockBackend final : public RobotBackend {
 public:
  explicit ScriptedMockBackend(MockScenario scenario);

  std::string Name() const override;
  std::string Version() const override;
  void BeginProgram() noexcept override;
  void Connect() override;
  void SetServoEnabled(bool enabled) override;
  MotionSubmission SubmitMoveJ(const MoveJRequest& request) override;
  MotionSubmission ResumeMoveJ(const MoveJRequest& request) override;
  MotionSnapshot PollMotion() override;
  StopSubmission SubmitStop(StopReason reason) override;
  StopSnapshot PollStop() override;
  void Disconnect() override;

  const std::vector<std::string>& call_log() const { return call_log_; }
  int stop_requests() const { return stop_requests_; }
  const JointPositions& positions() const { return positions_; }

 private:
  MotionSubmission SubmitMotion(const MoveJRequest& request, bool resume);
  MockScenario scenario_;
  bool connected_ = false;
  bool servo_enabled_ = false;
  bool motion_submitted_ = false;
  bool motion_frozen_ = false;
  std::size_t movej_occurrence_ = 0;
  MoveJRequest last_request_{};
  JointPositions positions_{};
  int stop_requests_ = 0;
  bool stop_submitted_ = false;
  bool stop_failed_ = false;
  bool stop_frozen_ = false;
  BusinessTime stop_delay_ms_ = 0;
  int stop_polls_ = 0;
  std::vector<std::string> call_log_;
};

}  // namespace roborun
