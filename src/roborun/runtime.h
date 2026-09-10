#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <vector>

#include "roborun/clock.h"
#include "roborun/config.h"
#include "roborun/event_queue.h"
#include "roborun/robot_backend.h"
#include "roborun/scenario.h"
#include "roborun/types.h"
#include "roborun/workcell_backend.h"

namespace roborun {

enum class ExecutionAdvanceResult {
  kIdle,
  kProgressed,
  kTerminal,
};

struct RuntimeOptions {
  BusinessTime move_timeout_ms = 30000;
  BusinessTime tool_timeout_ms = 500;
  BusinessTime grip_timeout_ms = 500;
  BusinessTime workcell_output_timeout_ms = 1000;
  BusinessTime workcell_safe_state_timeout_ms = 500;
  BusinessTime stop_confirmation_timeout_ms = 2000;
  BusinessTime default_stop_poll_ms = 10;
  bool disconnect_on_terminal = true;
};

class RuntimeExecution {
 public:
  RuntimeExecution(RobotBackend* backend, Clock* clock,
                   const RobotConfiguration* robot_configuration = nullptr,
                   const PointCatalog* point_catalog = nullptr,
                   const IOConfiguration* io_configuration = nullptr,
                   const ToolConfiguration* tool_configuration = nullptr,
                   const MockScenario* scenario = nullptr, RuntimeOptions options = {},
                   WorkcellBackend* workcell_backend = nullptr);

  ProgramSubmission SubmitProgram(const Program& program);
  ControlSubmission SubmitControl(ControlType type, std::string source = {});
  ControlSubmission SubmitControlWithRequestId(ControlType type, std::uint64_t request_id,
                                               std::string source = {});
  ControlSubmission SubmitControlAt(BusinessTime time, ControlType type, std::string source = {});
  ExecutionAdvanceResult Advance();

  bool is_terminal() const { return task_terminal_; }
  const std::optional<ActiveCommand>& active_command() const { return active_command_; }
  LifecycleState lifecycle_state() const { return lifecycle_state_; }
  BackendState backend_state() const { return backend_state_; }
  BackendOperationState backend_operation_state() const { return backend_operation_state_; }
  const ExecutionResult& result() const { return result_; }
  RuntimeSnapshot Snapshot() const;

 private:
  enum class StopDisposition {
    kPause,
    kProgramComplete,
    kOperatorStop,
    kEmergency,
    kFault,
  };

  struct StopEpisode {
    std::uint64_t id = 0;
    StopReason reason = StopReason::kProgramComplete;
    StopDisposition disposition = StopDisposition::kProgramComplete;
    BusinessTime deadline_ms = 0;
    std::optional<RuntimeEvent> control_event;
  };

  void PrepareResult(const Program& program);
  void SynchronizeResultState();
  void RefreshSnapshot();
  RuntimeSnapshot BuildSnapshot() const;
  void CopyWorkcellState();
  bool ValidateScenario(std::vector<Diagnostic>* diagnostics) const;
  void AdvanceClockTo(BusinessTime time);
  void EnsureConnected();

  void AppendTrace(const Command& command, std::size_t command_index, CommandExecutionStatus status,
                   RuntimeState before, RuntimeState during, RuntimeState after,
                   const std::string& cause, const MotionSnapshot* snapshot = nullptr);
  void AppendControlTrace(const RuntimeEvent& event, ControlSubmissionStatus status,
                          const std::string& cause, LifecycleState before,
                          std::optional<StopReason> stop_reason = std::nullopt,
                          std::optional<StopStatus> stop_status = std::nullopt);
  void RaiseAlarm(DiagnosticCode code, AlarmSeverity severity, std::string source,
                  std::string reason, std::string details, bool recoverable,
                  std::vector<AlarmRecoveryCondition> recovery_conditions = {});
  void AddDiagnostic(DiagnosticCode code, const std::string& message);

  void ScheduleCommandTimeout(const ActiveCommand& active);
  void ScheduleMotionPoll(BusinessTime delay_ms, std::uint64_t generation);
  void ApplyWorkcellSnapshot(const WorkcellSnapshot& snapshot);
  void BeginStop(StopReason reason, StopDisposition disposition,
                 const std::optional<RuntimeEvent>& control_event = std::nullopt);
  void HandleStopPoll(const RuntimeEvent& event);
  void HandleStopConfirmed(const StopSnapshot& snapshot);
  void FailStop(DiagnosticCode code, const std::string& message);
  void BeginFault(DiagnosticCode code, const std::string& message, bool recoverable,
                  StopReason stop_reason = StopReason::kFaultCleanup);
  void FinishFault();
  void FinishTerminal(ExecutionOutcome outcome);
  void DisableServoForCleanup();
  void DisconnectForCleanup();

  void FinishActiveSuccess(const std::string& cause, const MotionSnapshot* snapshot = nullptr);
  void HandleExternalEvent(const RuntimeEvent& event);
  void HandleTimeout(const RuntimeEvent& event);
  void HandleMotionPoll(const RuntimeEvent& event);
  void HandleDelayComplete(const RuntimeEvent& event);
  void HandleControl(const RuntimeEvent& event);
  void HandleNextCommand();
  void ResumeActiveCommand();
  void PauseLocalCommand(const RuntimeEvent& event);
  void ResetSession(const RuntimeEvent& event);
  void HandleEvent(const RuntimeEvent& event);

  Program program_;
  RobotBackend* backend_ = nullptr;
  Clock* clock_ = nullptr;
  const RobotConfiguration* robot_configuration_ = nullptr;
  const PointCatalog* point_catalog_ = nullptr;
  const IOConfiguration* io_configuration_ = nullptr;
  const ToolConfiguration* tool_configuration_ = nullptr;
  const MockScenario* scenario_ = nullptr;
  RuntimeOptions options_;
  WorkcellBackend* workcell_backend_ = nullptr;
  EventQueue events_;
  ExecutionResult result_;
  std::vector<Alarm> alarm_history_;
  RuntimeState state_ = RuntimeState::kServoOff;
  LifecycleState lifecycle_state_ = LifecycleState::kIdle;
  BackendState backend_state_ = BackendState::kDisconnected;
  BackendOperationState backend_operation_state_ = BackendOperationState::kIdle;
  JointPositions latest_positions_{};
  std::map<std::string, DigitalSignal> signals_;
  std::map<std::string, ToolState> tools_;
  std::size_t next_command_index_ = 0;
  std::optional<ActiveCommand> active_command_;
  std::optional<StopEpisode> stop_episode_;
  bool connected_ = false;
  bool servo_disabled_ = true;
  bool task_terminal_ = false;
  bool session_untrusted_ = false;
  bool workcell_safe_requested_ = false;
  bool workcell_safe_confirmed_ = false;
  std::optional<BusinessTime> workcell_safe_deadline_ms_;
  std::optional<std::uint64_t> last_workcell_generation_;
  std::optional<std::uint64_t> workcell_safe_generation_;
  std::uint64_t next_control_request_id_ = 1;
  std::uint64_t next_stop_episode_id_ = 1;
  std::uint64_t snapshot_sequence_ = 0;
  std::string snapshot_fingerprint_;
};

using RuntimeSession = RuntimeExecution;

class RobotTaskRuntime {
 public:
  ExecutionResult Execute(const Program& program, RobotBackend* backend) const;
  ExecutionResult Execute(const Program& program, const RobotConfiguration* robot_configuration,
                          const PointCatalog* point_catalog, RobotBackend* backend) const;
  ExecutionResult Execute(const Program& program, const RobotConfiguration* robot_configuration,
                          const PointCatalog* point_catalog,
                          const IOConfiguration* io_configuration,
                          const ToolConfiguration* tool_configuration, const MockScenario* scenario,
                          RobotBackend* backend, WorkcellBackend* workcell_backend = nullptr) const;

  RuntimeExecution Start(const Program& program, RobotBackend* backend, Clock* clock) const;
  RuntimeExecution Start(const Program& program, const RobotConfiguration* robot_configuration,
                         const PointCatalog* point_catalog, RobotBackend* backend,
                         Clock* clock) const;
  RuntimeExecution Start(const Program& program, const RobotConfiguration* robot_configuration,
                         const PointCatalog* point_catalog, const IOConfiguration* io_configuration,
                         const ToolConfiguration* tool_configuration, const MockScenario* scenario,
                         RobotBackend* backend, Clock* clock,
                         WorkcellBackend* workcell_backend = nullptr) const;
};

}  // namespace roborun
