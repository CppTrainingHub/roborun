#pragma once

#include <memory>
#include <optional>
#include <string>

#include "roborun/robot_backend.h"
#include "roborun/simulator_lifecycle.h"
#include "roborun/workcell.h"
#include "roborun/workcell_backend.h"

namespace roborun {

struct CoppeliaSimConfig {
  std::string host = "127.0.0.1";
  int port = 23000;
  std::string model_path;
  std::string resources_path;
  std::string scene_path;
  std::optional<WorkcellConfiguration> workcell;
};

class CoppeliaSimBackend final : public RobotBackend,
                                 public WorkcellBackend,
                                 public SimulatorLifecycle {
 public:
  explicit CoppeliaSimBackend(CoppeliaSimConfig config);
  ~CoppeliaSimBackend() override;

  std::string Name() const override;
  std::string Version() const override;
  void Connect() override;
  void SetServoEnabled(bool enabled) override;
  MotionSubmission SubmitMoveJ(const MoveJRequest& request) override;
  MotionSnapshot PollMotion() override;
  StopSubmission SubmitStop(StopReason reason) override;
  StopSnapshot PollStop() override;
  void Disconnect() override;
  WorkcellCommandSubmission SubmitToolCommand(const std::string& name,
                                              ToolPosition position) override;
  WorkcellCommandSubmission SetDigitalOutput(const std::string& name, bool value) override;
  WorkcellSnapshot PollWorkcell() override;
  WorkcellCommandSubmission RequestSafeState(StopReason reason) override;
  WorkcellCommandSubmission PollSafeState() override;

  void StopSimulation() override;
  void QuitSimulator() override;

 private:
  struct Impl;

  JointPositions ReadJointPositions() const;
  WorkcellSnapshot ReadWorkcellSnapshot();
  void UpdateToolMount();
  void BuildWorkcell();
  void ValidateConnectedWorkcell();

  CoppeliaSimConfig config_;
  std::unique_ptr<Impl> impl_;
  bool servo_enabled_ = false;
};

}  // namespace roborun
