#include "roborun/runtime.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "roborun/validator.h"

namespace roborun {
namespace {

constexpr std::size_t kNoCommand = std::numeric_limits<std::size_t>::max();

std::string UtcTimestamp() {
  const std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm utc{};
#if defined(_WIN32)
  gmtime_s(&utc, &time);
#else
  gmtime_r(&time, &utc);
#endif
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

bool IsLocalWait(CommandType type) {
  return type == CommandType::kDelay || type == CommandType::kWaitDi ||
         type == CommandType::kSetTool;
}

std::optional<BusinessTime> CheckedBusinessTimeAdd(BusinessTime base, BusinessTime duration) {
  if (base < 0 || duration < 0 || base > std::numeric_limits<BusinessTime>::max() - duration) {
    return std::nullopt;
  }
  return base + duration;
}

BusinessTime SaturatingBusinessTimeAdd(BusinessTime base, BusinessTime duration) {
  return CheckedBusinessTimeAdd(base, duration).value_or(std::numeric_limits<BusinessTime>::max());
}

}  // namespace

RuntimeExecution::RuntimeExecution(RobotBackend* backend, Clock* clock,
                                   const RobotConfiguration* robot_configuration,
                                   const PointCatalog* point_catalog,
                                   const IOConfiguration* io_configuration,
                                   const ToolConfiguration* tool_configuration,
                                   const MockScenario* scenario, RuntimeOptions options,
                                   WorkcellBackend* workcell_backend)
    : backend_(backend),
      clock_(clock),
      robot_configuration_(robot_configuration),
      point_catalog_(point_catalog),
      io_configuration_(io_configuration),
      tool_configuration_(tool_configuration),
      scenario_(scenario),
      options_(options),
      workcell_backend_(workcell_backend) {
  CopyWorkcellState();
  SynchronizeResultState();
}

void RuntimeExecution::PrepareResult(const Program& program) {
  program_ = program;
  events_.Clear();
  result_ = {};
  result_.task_source = program.source;
  result_.started_at = UtcTimestamp();
  result_.alarms = alarm_history_;
  for (std::size_t index = 0; index < result_.alarms.size(); ++index) {
    if (result_.alarms[index].active) {
      result_.primary_alarm_index = index;
      break;
    }
  }
  state_ = RuntimeState::kServoOff;
  lifecycle_state_ = LifecycleState::kRunning;
  backend_state_ = BackendState::kDisconnected;
  backend_operation_state_ = BackendOperationState::kIdle;
  latest_positions_ = {};
  next_command_index_ = 0;
  active_command_.reset();
  stop_episode_.reset();
  connected_ = false;
  servo_disabled_ = true;
  task_terminal_ = false;
  workcell_safe_requested_ = false;
  workcell_safe_confirmed_ = false;
  workcell_safe_deadline_ms_.reset();
  last_workcell_generation_.reset();
  workcell_safe_generation_.reset();
  CopyWorkcellState();
  SynchronizeResultState();
}

void RuntimeExecution::SynchronizeResultState() {
  result_.lifecycle_state = lifecycle_state_;
  result_.backend_state = backend_state_;
  result_.backend_operation_state = backend_operation_state_;
}

RuntimeSnapshot RuntimeExecution::BuildSnapshot() const {
  RuntimeSnapshot snapshot;
  snapshot.sequence = snapshot_sequence_;
  snapshot.runtime_state = state_;
  snapshot.lifecycle_state = lifecycle_state_;
  snapshot.backend_state = backend_state_;
  snapshot.backend_operation_state = backend_operation_state_;
  snapshot.active_command = active_command_;
  snapshot.business_time_ms = clock_ == nullptr ? 0 : clock_->Now();
  snapshot.joint_positions = latest_positions_;
  snapshot.signals = signals_;
  snapshot.tools = tools_;
  snapshot.alarms = result_.alarms;
  snapshot.primary_alarm_index = result_.primary_alarm_index;
  if (task_terminal_) {
    snapshot.terminal =
        RuntimeTerminalSummary{result_.succeeded, result_.outcome, result_.final_state,
                               result_.finished_business_time_ms, result_.diagnostics};
  }
  return snapshot;
}

RuntimeSnapshot RuntimeExecution::Snapshot() const { return BuildSnapshot(); }

void RuntimeExecution::RefreshSnapshot() {
  const RuntimeSnapshot snapshot = BuildSnapshot();
  std::ostringstream fingerprint;
  fingerprint << static_cast<int>(snapshot.runtime_state) << '|'
              << static_cast<int>(snapshot.lifecycle_state) << '|'
              << static_cast<int>(snapshot.backend_state) << '|'
              << static_cast<int>(snapshot.backend_operation_state) << '|'
              << snapshot.business_time_ms << '|';
  if (snapshot.active_command.has_value()) {
    const ActiveCommand& command = *snapshot.active_command;
    fingerprint << command.command_index << ':' << command.line << ':'
                << static_cast<int>(command.type) << ':' << static_cast<int>(command.status) << ':'
                << command.started_at_ms << ':' << command.remaining_ms << ':' << command.generation
                << ':';
    if (command.deadline_ms.has_value()) {
      fingerprint << *command.deadline_ms;
    }
  }
  for (double position : snapshot.joint_positions) {
    fingerprint << '|' << position;
  }
  for (const auto& [name, signal] : snapshot.signals) {
    fingerprint << '|' << name << ':' << static_cast<int>(signal.direction) << ':' << signal.value
                << ':' << signal.last_update_ms;
  }
  for (const auto& [name, tool] : snapshot.tools) {
    fingerprint << '|' << name << ':' << static_cast<int>(tool.type) << ':'
                << static_cast<int>(tool.commanded) << ':' << static_cast<int>(tool.observed) << ':'
                << tool.last_feedback_ms;
  }
  for (const Alarm& alarm : snapshot.alarms) {
    fingerprint << '|' << static_cast<int>(alarm.code) << ':' << static_cast<int>(alarm.severity)
                << ':' << alarm.source << ':' << alarm.reason << ':' << alarm.details << ':'
                << alarm.raised_business_time_ms << ':' << alarm.recoverable << ':' << alarm.active;
    for (AlarmRecoveryCondition condition : alarm.recovery_conditions) {
      fingerprint << ':' << static_cast<int>(condition);
    }
  }
  if (snapshot.primary_alarm_index.has_value()) {
    fingerprint << "|primary:" << *snapshot.primary_alarm_index;
  }
  if (snapshot.terminal.has_value()) {
    fingerprint << "|terminal:" << snapshot.terminal->succeeded << ':'
                << static_cast<int>(snapshot.terminal->outcome) << ':'
                << static_cast<int>(snapshot.terminal->final_state) << ':'
                << snapshot.terminal->finished_business_time_ms;
    for (const Diagnostic& diagnostic : snapshot.terminal->diagnostics) {
      fingerprint << ':' << static_cast<int>(diagnostic.code) << ':' << diagnostic.source << ':'
                  << diagnostic.line << ':' << diagnostic.column << ':' << diagnostic.path << ':'
                  << diagnostic.message;
    }
  }
  if (fingerprint.str() != snapshot_fingerprint_) {
    snapshot_fingerprint_ = fingerprint.str();
    ++snapshot_sequence_;
  }
}

void RuntimeExecution::CopyWorkcellState() {
  if (io_configuration_ != nullptr) {
    signals_ = io_configuration_->signals;
  }
  if (tool_configuration_ != nullptr) {
    tools_ = tool_configuration_->tools;
  }
  result_.signals = signals_;
  result_.tools = tools_;
}

void RuntimeExecution::AddDiagnostic(DiagnosticCode code, const std::string& message) {
  result_.diagnostics.push_back(Diagnostic{code, program_.source, 0, 0, "", message});
}

void RuntimeExecution::RaiseAlarm(DiagnosticCode code, AlarmSeverity severity, std::string source,
                                  std::string reason, std::string details, bool recoverable,
                                  std::vector<AlarmRecoveryCondition> recovery_conditions) {
  Alarm alarm{code,
              severity,
              std::move(source),
              std::move(reason),
              std::move(details),
              clock_ == nullptr ? 0 : clock_->Now(),
              recoverable,
              true,
              std::move(recovery_conditions)};
  alarm_history_.push_back(alarm);
  result_.alarms.push_back(std::move(alarm));
  if (!result_.primary_alarm_index.has_value()) {
    result_.primary_alarm_index = result_.alarms.size() - 1;
  }
  AddDiagnostic(code, result_.alarms.back().details);
}

bool RuntimeExecution::ValidateScenario(std::vector<Diagnostic>* diagnostics) const {
  if (scenario_ == nullptr) {
    return true;
  }
  std::size_t movej_count = 0;
  for (const Command& command : program_.commands) {
    if (command.type == CommandType::kMoveJ) {
      ++movej_count;
    }
  }
  bool valid = scenario_->schema_version == 1 || scenario_->schema_version == 2;
  if (!valid) {
    diagnostics->push_back(Diagnostic{DiagnosticCode::kInvalidScenario, program_.source, 0, 0, "",
                                      "scenario schema version is unsupported"});
  }
  for (const auto& [occurrence, plan] : scenario_->motion_plans) {
    if (occurrence == 0 || occurrence > movej_count || plan.movej_occurrence != occurrence) {
      diagnostics->push_back(Diagnostic{DiagnosticCode::kInvalidScenario, program_.source, 0, 0, "",
                                        "scenario references an invalid MOVEJ occurrence"});
      valid = false;
    }
  }
  BusinessTime previous_time = 0;
  for (const ScenarioEvent& event : scenario_->events) {
    if (event.time_ms < previous_time ||
        (event.kind == ScenarioEventKind::kConnectionLoss && event.time_ms == 0)) {
      diagnostics->push_back(Diagnostic{DiagnosticCode::kInvalidScenario, program_.source, 0, 0, "",
                                        "scenario event time is invalid"});
      valid = false;
    }
    previous_time = event.time_ms;
    if (event.kind == ScenarioEventKind::kDiChange &&
        (io_configuration_ == nullptr || !io_configuration_->signals.contains(event.name) ||
         io_configuration_->signals.at(event.name).direction != SignalDirection::kDi)) {
      diagnostics->push_back(
          Diagnostic{DiagnosticCode::kInvalidScenario, program_.source, 0, 0, "",
                     "scenario DI event references an unknown or non-DI signal"});
      valid = false;
    }
    if (event.kind == ScenarioEventKind::kToolFeedback &&
        (tool_configuration_ == nullptr || !tool_configuration_->tools.contains(event.name))) {
      diagnostics->push_back(Diagnostic{DiagnosticCode::kInvalidScenario, program_.source, 0, 0, "",
                                        "scenario tool event references an unknown tool"});
      valid = false;
    }
  }
  previous_time = 0;
  for (const ScenarioControl& control : scenario_->controls) {
    if (scenario_->schema_version != 2 || control.time_ms < previous_time ||
        control.type == ControlType::kNone) {
      diagnostics->push_back(Diagnostic{DiagnosticCode::kInvalidScenario, program_.source, 0, 0, "",
                                        "scenario control is invalid"});
      valid = false;
    }
    previous_time = control.time_ms;
  }
  for (const auto& [episode, plan] : scenario_->stop_plans) {
    if (scenario_->schema_version != 2 || episode == 0 || plan.stop_episode != episode) {
      diagnostics->push_back(Diagnostic{DiagnosticCode::kInvalidScenario, program_.source, 0, 0, "",
                                        "scenario stop plan is invalid"});
      valid = false;
    }
  }
  return valid;
}

ProgramSubmission RuntimeExecution::SubmitProgram(const Program& program) {
  const bool replaceable_rejected_submission =
      task_terminal_ && lifecycle_state_ == LifecycleState::kFaulted && result_.alarms.empty() &&
      !connected_ && !active_command_.has_value() && !stop_episode_.has_value();
  if (lifecycle_state_ != LifecycleState::kIdle && lifecycle_state_ != LifecycleState::kStopped &&
      !replaceable_rejected_submission) {
    return {ProgramSubmissionStatus::kRejected, "Runtime Session already has an active Program"};
  }
  if (backend_ == nullptr || clock_ == nullptr) {
    return {ProgramSubmissionStatus::kRejected,
            backend_ == nullptr ? "robot backend is missing" : "clock is missing"};
  }
  ValidationResult validation = ValidateProgram(program, robot_configuration_, point_catalog_,
                                                io_configuration_, tool_configuration_);
  if (robot_configuration_ != nullptr && point_catalog_ != nullptr) {
    std::vector<Diagnostic> catalog_diagnostics =
        ValidatePointCatalog(*point_catalog_, *robot_configuration_);
    validation.diagnostics.insert(validation.diagnostics.end(), catalog_diagnostics.begin(),
                                  catalog_diagnostics.end());
  }
  if (!validation.diagnostics.empty()) {
    PrepareResult(program);
    state_ = RuntimeState::kFaulted;
    lifecycle_state_ = LifecycleState::kFaulted;
    task_terminal_ = true;
    result_.diagnostics = validation.diagnostics;
    result_.outcome = ExecutionOutcome::kFaulted;
    result_.final_state = state_;
    result_.finished_at = UtcTimestamp();
    SynchronizeResultState();
    RefreshSnapshot();
    return {ProgramSubmissionStatus::kRejected, "Program validation failed"};
  }
  PrepareResult(validation.program);
  std::vector<Diagnostic> scenario_diagnostics;
  if (!ValidateScenario(&scenario_diagnostics)) {
    state_ = RuntimeState::kFaulted;
    lifecycle_state_ = LifecycleState::kFaulted;
    task_terminal_ = true;
    result_.diagnostics = std::move(scenario_diagnostics);
    result_.outcome = ExecutionOutcome::kFaulted;
    result_.final_state = state_;
    result_.finished_at = UtcTimestamp();
    SynchronizeResultState();
    RefreshSnapshot();
    return {ProgramSubmissionStatus::kRejected, "Mock scenario validation failed"};
  }
  result_.backend_name = backend_->Name();
  result_.backend_version = backend_->Version();
  backend_->BeginProgram();
  if (scenario_ != nullptr) {
    for (const ScenarioEvent& event : scenario_->events) {
      if (event.kind == ScenarioEventKind::kDiChange) {
        events_.Schedule({event.time_ms,
                          RuntimeEventPriority::kExternalFeedback,
                          RuntimeEventType::kDigitalInput,
                          event.name,
                          event.value,
                          {}});
      } else if (event.kind == ScenarioEventKind::kToolFeedback) {
        events_.Schedule({event.time_ms, RuntimeEventPriority::kExternalFeedback,
                          RuntimeEventType::kToolFeedback, event.name, false,
                          ToString(event.tool_position)});
      } else {
        events_.Schedule({event.time_ms,
                          RuntimeEventPriority::kExternalFeedback,
                          RuntimeEventType::kBackendFault,
                          {},
                          false,
                          "connection loss"});
      }
    }
    for (const ScenarioControl& control : scenario_->controls) {
      SubmitControlAt(control.time_ms, control.type, control.source);
    }
  }
  RefreshSnapshot();
  return {ProgramSubmissionStatus::kAccepted, {}};
}

ControlSubmission RuntimeExecution::SubmitControl(ControlType type, std::string source) {
  return SubmitControlWithRequestId(type, next_control_request_id_++, std::move(source));
}

ControlSubmission RuntimeExecution::SubmitControlWithRequestId(ControlType type,
                                                               std::uint64_t request_id,
                                                               std::string source) {
  if (request_id == 0 || type == ControlType::kNone || clock_ == nullptr) {
    return {request_id, type, ControlSubmissionStatus::kRejected,
            request_id == 0              ? "control request id is missing"
            : type == ControlType::kNone ? "control type is missing"
                                         : "clock is missing"};
  }
  next_control_request_id_ = std::max(next_control_request_id_, request_id + 1);
  events_.ScheduleControl(clock_->Now(), type, request_id, std::move(source));
  return {request_id, type, ControlSubmissionStatus::kQueued, {}};
}

ControlSubmission RuntimeExecution::SubmitControlAt(BusinessTime time, ControlType type,
                                                    std::string source) {
  if (type == ControlType::kNone || clock_ == nullptr || time < clock_->Now()) {
    return {0, type, ControlSubmissionStatus::kRejected,
            type == ControlType::kNone ? "control type is missing" : "control time is invalid"};
  }
  const std::uint64_t request_id = next_control_request_id_++;
  events_.ScheduleControl(time, type, request_id, std::move(source));
  return {request_id, type, ControlSubmissionStatus::kQueued, {}};
}

void RuntimeExecution::AdvanceClockTo(BusinessTime time) {
  if (time > clock_->Now()) {
    clock_->AdvanceTo(time);
  }
}

void RuntimeExecution::EnsureConnected() {
  if (connected_) {
    return;
  }
  try {
    backend_->Connect();
  } catch (...) {
    try {
      backend_->Disconnect();
    } catch (...) {
    }
    throw;
  }
  connected_ = true;
  backend_state_ = BackendState::kConnected;
}

void RuntimeExecution::AppendTrace(const Command& command, std::size_t command_index,
                                   CommandExecutionStatus status, RuntimeState before,
                                   RuntimeState during, RuntimeState after,
                                   const std::string& cause, const MotionSnapshot* snapshot) {
  TraceEntry entry;
  entry.business_time_ms = clock_->Now();
  entry.command_index = command_index + 1;
  entry.line = command.line;
  entry.command = command.type;
  entry.state_before = before;
  entry.state_during = during;
  entry.state_after = after;
  entry.lifecycle_before = lifecycle_state_;
  entry.lifecycle_during = lifecycle_state_;
  entry.lifecycle_after = lifecycle_state_;
  entry.command_status = status;
  entry.event_cause = cause;
  entry.point_name = command.movej.point_name;
  entry.signal_name = command.signal_name;
  entry.tool_name = command.tool_name;
  entry.expected_value = command.signal_value;
  entry.speed_percent = command.type == CommandType::kMoveJ ? command.movej.speed_percent : 0;
  entry.targets = command.movej.targets;
  entry.actual_positions = latest_positions_;
  entry.outcome = result_.outcome;
  if (active_command_.has_value() && active_command_->command_index == command_index) {
    entry.deadline_ms = active_command_->deadline_ms.value_or(-1);
  }
  if (command.type == CommandType::kSetTool) {
    entry.commanded_tool_position = command.tool_position;
    const auto tool = tools_.find(command.tool_name);
    if (tool != tools_.end()) {
      entry.observed_tool_position = tool->second.observed;
    }
  }
  if (command.type == CommandType::kSetDo || command.type == CommandType::kWaitDi) {
    const auto signal = signals_.find(command.signal_name);
    if (signal != signals_.end()) {
      entry.observed_value = signal->second.value;
    }
  }
  if (snapshot != nullptr) {
    entry.backend_steps = snapshot->backend_steps;
    entry.actual_positions = snapshot->positions;
  }
  entry.workcell_generation = last_workcell_generation_.value_or(0);
  result_.trace.push_back(std::move(entry));
}

void RuntimeExecution::AppendControlTrace(const RuntimeEvent& event, ControlSubmissionStatus status,
                                          const std::string& cause, LifecycleState before,
                                          std::optional<StopReason> stop_reason,
                                          std::optional<StopStatus> stop_status) {
  TraceEntry entry{};
  entry.business_time_ms = clock_->Now();
  entry.command_index = active_command_.has_value() ? active_command_->command_index + 1 : 0;
  entry.state_before = state_;
  entry.state_during = state_;
  entry.state_after = state_;
  entry.lifecycle_before = before;
  entry.lifecycle_during = lifecycle_state_;
  entry.lifecycle_after = lifecycle_state_;
  entry.control_type = event.control_type;
  entry.control_status = status;
  entry.control_request_id = event.control_request_id;
  entry.control_source = event.control_source;
  entry.event_cause = cause;
  entry.outcome = result_.outcome;
  if (active_command_.has_value()) {
    entry.command = active_command_->type;
    entry.line = active_command_->line;
  }
  entry.workcell_generation = last_workcell_generation_.value_or(0);
  if (stop_reason.has_value()) {
    entry.stop_reason = *stop_reason;
  }
  if (stop_status.has_value()) {
    entry.stop_status = *stop_status;
  }
  result_.trace.push_back(std::move(entry));
}

void RuntimeExecution::ScheduleCommandTimeout(const ActiveCommand& active) {
  if (active.deadline_ms.has_value()) {
    events_.Schedule({*active.deadline_ms,
                      RuntimeEventPriority::kTimeout,
                      RuntimeEventType::kBackendFault,
                      {},
                      false,
                      "timeout",
                      active.command_index,
                      active.generation});
  }
}

void RuntimeExecution::ScheduleMotionPoll(BusinessTime delay_ms, std::uint64_t generation) {
  BusinessTime poll_time =
      SaturatingBusinessTimeAdd(clock_->Now(), std::max<BusinessTime>(0, delay_ms));
  if (active_command_.has_value() && active_command_->deadline_ms.has_value()) {
    poll_time = std::min(poll_time, *active_command_->deadline_ms);
  }
  events_.Schedule({poll_time,
                    RuntimeEventPriority::kExternalFeedback,
                    RuntimeEventType::kMotionPoll,
                    {},
                    false,
                    {},
                    active_command_.has_value() ? active_command_->command_index : kNoCommand,
                    generation});
}

void RuntimeExecution::ApplyWorkcellSnapshot(const WorkcellSnapshot& snapshot) {
  if (!snapshot.valid) {
    BeginFault(
        DiagnosticCode::kWorkcellSnapshotFailure,
        snapshot.message.empty() ? "workcell returned an invalid snapshot" : snapshot.message,
        false);
    return;
  }
  if (last_workcell_generation_.has_value() && snapshot.generation <= *last_workcell_generation_) {
    return;
  }
  last_workcell_generation_ = snapshot.generation;
  latest_positions_ = snapshot.joint_positions;
  signals_ = snapshot.signals;
  tools_ = snapshot.tools;
  result_.signals = signals_;
  result_.tools = tools_;
  WorkcellEvidence evidence;
  evidence.generation = snapshot.generation;
  evidence.scene_schema = snapshot.scene_schema;
  evidence.joint_positions = snapshot.joint_positions;
  evidence.signals = snapshot.signals;
  evidence.tools = snapshot.tools;
  evidence.tool_openings = snapshot.tool_openings;
  evidence.workpiece_present = snapshot.workpiece.present;
  evidence.workpiece_attached = snapshot.workpiece.attached;
  evidence.workpiece_at_place = snapshot.workpiece.at_place;
  evidence.workpiece_pose = snapshot.workpiece.world_pose;
  evidence.collision = snapshot.collision;
  evidence.collision_aliases = snapshot.collision_aliases;
  evidence.simulation_running = snapshot.simulation_running;
  result_.workcell = std::move(evidence);
  if (snapshot.collision) {
    std::ostringstream details;
    details << "workcell generation " << snapshot.generation << " collision";
    for (const std::string& aliases : snapshot.collision_aliases) {
      details << ' ' << aliases;
    }
    if (active_command_.has_value()) {
      const Command& command = program_.commands[active_command_->command_index];
      active_command_->status = CommandExecutionStatus::kFailed;
      AppendTrace(command, active_command_->command_index, CommandExecutionStatus::kFailed, state_,
                  state_, RuntimeState::kFaulted, "workcell_collision");
    }
    BeginFault(DiagnosticCode::kCollisionDetected, details.str(), false);
    return;
  }
  if (!active_command_.has_value() || (lifecycle_state_ != LifecycleState::kRunning &&
                                       lifecycle_state_ != LifecycleState::kWaiting)) {
    return;
  }
  const Command& command = program_.commands[active_command_->command_index];
  if (command.type == CommandType::kWaitDi && signals_.contains(command.signal_name) &&
      signals_.at(command.signal_name).value == command.signal_value) {
    FinishActiveSuccess("workcell_di_satisfied");
  } else if (command.type == CommandType::kSetTool && tools_.contains(command.tool_name) &&
             tools_.at(command.tool_name).observed == command.tool_position) {
    const int tool_stability = snapshot.tool_stable_samples.contains(command.tool_name)
                                   ? snapshot.tool_stable_samples.at(command.tool_name)
                                   : 0;
    if (tool_stability < snapshot.required_stable_samples) {
      return;
    }
    if (command.tool_position == ToolPosition::kClosed && snapshot.grip_failed) {
      BeginFault(DiagnosticCode::kGripFailure, "gripper closed without attaching the workpiece",
                 true);
      return;
    }
    if (command.tool_position == ToolPosition::kOpen && snapshot.placement_failed) {
      BeginFault(DiagnosticCode::kPlacementFailure,
                 "gripper opened without a stable detached workpiece at PLACE", true);
      return;
    }
    if (command.tool_position == ToolPosition::kClosed &&
        (!snapshot.workpiece.attached ||
         snapshot.workpiece.attached_stable_samples < snapshot.required_stable_samples)) {
      return;
    }
    if (command.tool_position == ToolPosition::kOpen &&
        (snapshot.workpiece.attached ||
         snapshot.workpiece.place_stable_samples < snapshot.required_stable_samples)) {
      return;
    }
    FinishActiveSuccess("workcell_tool_satisfied");
  } else if (command.type == CommandType::kSetDo && signals_.contains(command.signal_name) &&
             signals_.at(command.signal_name).value == command.signal_value) {
    FinishActiveSuccess("workcell_do_satisfied");
  }
}

void RuntimeExecution::DisableServoForCleanup() {
  if (!connected_ || servo_disabled_) {
    return;
  }
  try {
    backend_->SetServoEnabled(false);
    servo_disabled_ = true;
  } catch (const std::exception& error) {
    session_untrusted_ = true;
    RaiseAlarm(DiagnosticCode::kServoDisableFailure, AlarmSeverity::kFault, "backend",
               "servo_disable_failed", error.what(), false,
               {AlarmRecoveryCondition::kNewBackendSession});
  }
}

void RuntimeExecution::DisconnectForCleanup() {
  if (!options_.disconnect_on_terminal) {
    return;
  }
  if (!connected_) {
    backend_state_ = BackendState::kDisconnected;
    return;
  }
  try {
    backend_->Disconnect();
    connected_ = false;
    backend_state_ = BackendState::kDisconnected;
  } catch (const std::exception& error) {
    session_untrusted_ = true;
    backend_state_ = BackendState::kFaulted;
    RaiseAlarm(DiagnosticCode::kDisconnectFailure, AlarmSeverity::kFault, "backend",
               "disconnect_failed", error.what(), false,
               {AlarmRecoveryCondition::kNewBackendSession});
  }
}

void RuntimeExecution::BeginFault(DiagnosticCode code, const std::string& message, bool recoverable,
                                  StopReason stop_reason) {
  if (!result_.primary_alarm_index.has_value()) {
    RaiseAlarm(
        code, AlarmSeverity::kFault, "runtime", ToString(code), message, recoverable,
        recoverable
            ? std::vector<AlarmRecoveryCondition>{AlarmRecoveryCondition::kNoActiveCommand,
                                                  AlarmRecoveryCondition::kBackendStopped,
                                                  AlarmRecoveryCondition::kServoDisabled}
            : std::vector<AlarmRecoveryCondition>{AlarmRecoveryCondition::kNewBackendSession});
  }
  if (workcell_backend_ != nullptr && !workcell_safe_requested_) {
    workcell_safe_requested_ = true;
    workcell_safe_deadline_ms_ =
        SaturatingBusinessTimeAdd(clock_->Now(), options_.workcell_safe_state_timeout_ms);
    const WorkcellCommandSubmission submission = workcell_backend_->RequestSafeState(stop_reason);
    if (submission.status == WorkcellCommandStatus::kRejected) {
      RaiseAlarm(DiagnosticCode::kBackendFailure, AlarmSeverity::kFault, "workcell",
                 "safe_state_rejected", submission.message, false,
                 {AlarmRecoveryCondition::kNewBackendSession});
    }
  }
  if (connected_ && !stop_episode_.has_value()) {
    BeginStop(stop_reason, StopDisposition::kFault);
    return;
  }
  FinishFault();
}

void RuntimeExecution::FinishFault() {
  DisableServoForCleanup();
  DisconnectForCleanup();
  active_command_.reset();
  stop_episode_.reset();
  state_ = RuntimeState::kFaulted;
  lifecycle_state_ = LifecycleState::kFaulted;
  backend_operation_state_ = BackendOperationState::kStopped;
  result_.succeeded = false;
  result_.outcome = ExecutionOutcome::kFaulted;
  result_.final_state = state_;
  result_.finished_business_time_ms = clock_ == nullptr ? 0 : clock_->Now();
  result_.signals = signals_;
  result_.tools = tools_;
  result_.finished_at = UtcTimestamp();
  task_terminal_ = true;
  SynchronizeResultState();
}

void RuntimeExecution::FinishTerminal(ExecutionOutcome outcome) {
  DisconnectForCleanup();
  if (session_untrusted_) {
    FinishFault();
    return;
  }
  state_ = RuntimeState::kStopped;
  lifecycle_state_ = outcome == ExecutionOutcome::kEmergencyStopped
                         ? LifecycleState::kEmergencyStopped
                         : LifecycleState::kStopped;
  backend_operation_state_ = BackendOperationState::kStopped;
  result_.succeeded = outcome == ExecutionOutcome::kProgramCompleted;
  result_.outcome = outcome;
  result_.final_state = state_;
  result_.finished_business_time_ms = clock_ == nullptr ? 0 : clock_->Now();
  result_.signals = signals_;
  result_.tools = tools_;
  result_.finished_at = UtcTimestamp();
  task_terminal_ = true;
  SynchronizeResultState();
}

void RuntimeExecution::FailStop(DiagnosticCode code, const std::string& message) {
  RaiseAlarm(code, AlarmSeverity::kFault, "backend", ToString(code), message, false,
             {AlarmRecoveryCondition::kNewBackendSession});
  session_untrusted_ = true;
  FinishFault();
}

void RuntimeExecution::BeginStop(StopReason reason, StopDisposition disposition,
                                 const std::optional<RuntimeEvent>& control_event) {
  if (stop_episode_.has_value()) {
    if (reason == StopReason::kEmergency &&
        stop_episode_->disposition != StopDisposition::kEmergency) {
      const LifecycleState lifecycle_before = lifecycle_state_;
      stop_episode_->reason = StopReason::kEmergency;
      stop_episode_->disposition = StopDisposition::kEmergency;
      lifecycle_state_ = LifecycleState::kEmergencyStopping;
      if (control_event.has_value()) {
        AppendControlTrace(*control_event, ControlSubmissionStatus::kAccepted,
                           "emergency_escalates_pending_stop", lifecycle_before, reason,
                           StopStatus::kStopping);
      }
      SynchronizeResultState();
      return;
    }
    if (control_event.has_value()) {
      AppendControlTrace(*control_event, ControlSubmissionStatus::kIgnored, "stop_already_pending",
                         lifecycle_state_, reason, StopStatus::kStopping);
    }
    return;
  }
  if (!connected_) {
    StopSnapshot stopped{StopStatus::kStopped, latest_positions_, 0, "", std::nullopt};
    stop_episode_ =
        StopEpisode{next_stop_episode_id_++, reason, disposition, clock_->Now(), control_event};
    HandleStopConfirmed(stopped);
    return;
  }
  const LifecycleState lifecycle_before = lifecycle_state_;
  StopSubmission submission;
  try {
    submission = backend_->SubmitStop(reason);
  } catch (const std::exception& error) {
    FailStop(DiagnosticCode::kStopPollingFailure, error.what());
    return;
  }
  if (submission.status == StopSubmissionStatus::kRejected) {
    FailStop(DiagnosticCode::kStopRejected,
             submission.message.empty() ? "backend rejected stop" : submission.message);
    return;
  }
  StopEpisode episode{
      next_stop_episode_id_++, reason, disposition,
      SaturatingBusinessTimeAdd(clock_->Now(), options_.stop_confirmation_timeout_ms),
      control_event};
  stop_episode_ = episode;
  backend_operation_state_ = BackendOperationState::kStopping;
  if (disposition == StopDisposition::kPause) {
    lifecycle_state_ = LifecycleState::kPausing;
  } else if (disposition == StopDisposition::kEmergency) {
    lifecycle_state_ = LifecycleState::kEmergencyStopping;
  } else {
    lifecycle_state_ = LifecycleState::kStopping;
  }
  if (control_event.has_value()) {
    AppendControlTrace(*control_event, ControlSubmissionStatus::kAccepted, "stop_submitted",
                       lifecycle_before, reason, StopStatus::kStopping);
  }
  const BusinessTime first_poll =
      std::min(episode.deadline_ms,
               SaturatingBusinessTimeAdd(
                   clock_->Now(), std::max<BusinessTime>(0, submission.poll_after_ms.value_or(0))));
  events_.Schedule({first_poll,
                    RuntimeEventPriority::kExternalFeedback,
                    RuntimeEventType::kStopPoll,
                    {},
                    false,
                    {},
                    kNoCommand,
                    0,
                    episode.id});
  events_.Schedule({episode.deadline_ms,
                    RuntimeEventPriority::kTimeout,
                    RuntimeEventType::kBackendFault,
                    {},
                    false,
                    "stop_timeout",
                    kNoCommand,
                    0,
                    episode.id});
  SynchronizeResultState();
}

void RuntimeExecution::HandleStopPoll(const RuntimeEvent& event) {
  if (!stop_episode_.has_value() || event.stop_episode_id != stop_episode_->id) {
    return;
  }
  StopSnapshot snapshot;
  try {
    snapshot = backend_->PollStop();
  } catch (const std::exception& error) {
    FailStop(DiagnosticCode::kStopPollingFailure, error.what());
    return;
  }
  latest_positions_ = snapshot.positions;
  if (snapshot.status == StopStatus::kFailed) {
    FailStop(DiagnosticCode::kStopPollingFailure,
             snapshot.message.empty() ? "backend stop polling failed" : snapshot.message);
    return;
  }
  if (snapshot.status == StopStatus::kStopping) {
    if (clock_->Now() >= stop_episode_->deadline_ms) {
      FailStop(DiagnosticCode::kStopConfirmationTimeout,
               "backend did not confirm stop before deadline");
      return;
    }
    const BusinessTime poll_delay =
        std::max<BusinessTime>(1, snapshot.poll_after_ms.value_or(options_.default_stop_poll_ms));
    events_.Schedule(
        {std::min(stop_episode_->deadline_ms, SaturatingBusinessTimeAdd(clock_->Now(), poll_delay)),
         RuntimeEventPriority::kExternalFeedback,
         RuntimeEventType::kStopPoll,
         {},
         false,
         {},
         kNoCommand,
         0,
         stop_episode_->id});
    return;
  }
  HandleStopConfirmed(snapshot);
}

void RuntimeExecution::HandleStopConfirmed(const StopSnapshot& snapshot) {
  if (!stop_episode_.has_value()) {
    return;
  }
  const StopEpisode episode = *stop_episode_;
  latest_positions_ = snapshot.positions;
  backend_operation_state_ = BackendOperationState::kStopped;
  stop_episode_.reset();
  if ((episode.disposition == StopDisposition::kFault ||
       episode.disposition == StopDisposition::kEmergency) &&
      workcell_backend_ != nullptr && workcell_safe_requested_) {
    if (workcell_safe_deadline_ms_.has_value() && clock_->Now() > *workcell_safe_deadline_ms_) {
      RaiseAlarm(DiagnosticCode::kBackendFailure, AlarmSeverity::kFault, "workcell",
                 "safe_state_timeout", "workcell safe state exceeded its configured deadline",
                 false, {AlarmRecoveryCondition::kNewBackendSession});
      session_untrusted_ = true;
    }
    const WorkcellCommandSubmission safe_snapshot = workcell_backend_->PollSafeState();
    if (safe_snapshot.status == WorkcellCommandStatus::kRejected) {
      RaiseAlarm(DiagnosticCode::kBackendFailure, AlarmSeverity::kFault, "workcell",
                 "safe_state_not_confirmed", safe_snapshot.message, false,
                 {AlarmRecoveryCondition::kNewBackendSession});
      session_untrusted_ = true;
    } else {
      workcell_safe_confirmed_ = true;
      workcell_safe_generation_ = last_workcell_generation_.value_or(0);
      result_.workcell_safe_confirmed = true;
    }
  }
  if (episode.disposition == StopDisposition::kPause) {
    if (active_command_.has_value()) {
      active_command_->status = CommandExecutionStatus::kPaused;
      ++active_command_->generation;
      const Command& command = program_.commands[active_command_->command_index];
      lifecycle_state_ = LifecycleState::kPaused;
      AppendTrace(command, active_command_->command_index, CommandExecutionStatus::kPaused, state_,
                  state_, state_, "pause_confirmed");
    }
    SynchronizeResultState();
    return;
  }
  if (episode.disposition == StopDisposition::kFault) {
    FinishFault();
    return;
  }
  if (active_command_.has_value()) {
    const Command& command = program_.commands[active_command_->command_index];
    const CommandExecutionStatus status = episode.disposition == StopDisposition::kProgramComplete
                                              ? CommandExecutionStatus::kSucceeded
                                              : CommandExecutionStatus::kCancelled;
    AppendTrace(command, active_command_->command_index, status, state_, state_,
                RuntimeState::kStopped, "stop_confirmed");
    if (episode.disposition == StopDisposition::kProgramComplete) {
      ++next_command_index_;
    }
    active_command_.reset();
  }
  if (episode.disposition == StopDisposition::kEmergency) {
    DisableServoForCleanup();
    if (session_untrusted_) {
      FinishFault();
      return;
    }
    RaiseAlarm(DiagnosticCode::kEmergencyStopped, AlarmSeverity::kEmergency, "runtime",
               "emergency_stop_confirmed", "emergency stop was confirmed", true,
               {AlarmRecoveryCondition::kNoActiveCommand, AlarmRecoveryCondition::kBackendStopped,
                AlarmRecoveryCondition::kServoDisabled});
    FinishTerminal(ExecutionOutcome::kEmergencyStopped);
    return;
  }
  FinishTerminal(episode.disposition == StopDisposition::kProgramComplete
                     ? ExecutionOutcome::kProgramCompleted
                     : ExecutionOutcome::kOperatorStopped);
}

void RuntimeExecution::FinishActiveSuccess(const std::string& cause,
                                           const MotionSnapshot* snapshot) {
  if (!active_command_.has_value() || (lifecycle_state_ != LifecycleState::kRunning &&
                                       lifecycle_state_ != LifecycleState::kWaiting)) {
    return;
  }
  const ActiveCommand active = *active_command_;
  const Command& command = program_.commands[active.command_index];
  AppendTrace(command, active.command_index, CommandExecutionStatus::kSucceeded, state_,
              RuntimeState::kRunning, RuntimeState::kReady, cause, snapshot);
  state_ = RuntimeState::kReady;
  lifecycle_state_ = LifecycleState::kRunning;
  backend_operation_state_ = BackendOperationState::kIdle;
  ++next_command_index_;
  active_command_.reset();
}

void RuntimeExecution::HandleExternalEvent(const RuntimeEvent& event) {
  if (event.type == RuntimeEventType::kBackendFault) {
    BeginFault(DiagnosticCode::kBackendConnectionLoss,
               event.detail.empty() ? "backend connection was lost" : event.detail, false);
    return;
  }
  if (event.type == RuntimeEventType::kDigitalInput) {
    const auto signal = signals_.find(event.name);
    if (signal == signals_.end()) {
      BeginFault(DiagnosticCode::kInvalidScenario, "scenario changed an unknown signal", false);
      return;
    }
    signal->second.value = event.value;
    signal->second.last_update_ms = clock_->Now();
    result_.signals = signals_;
    if (active_command_.has_value() && lifecycle_state_ != LifecycleState::kPaused &&
        active_command_->type == CommandType::kWaitDi &&
        event.name == program_.commands[active_command_->command_index].signal_name &&
        signal->second.value == program_.commands[active_command_->command_index].signal_value) {
      FinishActiveSuccess("di_satisfied");
    }
    return;
  }
  if (event.type == RuntimeEventType::kToolFeedback) {
    const auto tool = tools_.find(event.name);
    if (tool == tools_.end()) {
      BeginFault(DiagnosticCode::kInvalidScenario, "scenario changed an unknown tool", false);
      return;
    }
    tool->second.observed = event.detail == "CLOSED" ? ToolPosition::kClosed : ToolPosition::kOpen;
    tool->second.last_feedback_ms = clock_->Now();
    result_.tools = tools_;
    if (active_command_.has_value() && lifecycle_state_ != LifecycleState::kPaused &&
        active_command_->type == CommandType::kSetTool &&
        event.name == program_.commands[active_command_->command_index].tool_name &&
        tool->second.observed == program_.commands[active_command_->command_index].tool_position) {
      FinishActiveSuccess("tool_feedback_satisfied");
    }
  }
}

void RuntimeExecution::HandleTimeout(const RuntimeEvent& event) {
  if (event.detail == "stop_timeout") {
    if (stop_episode_.has_value() && event.stop_episode_id == stop_episode_->id &&
        clock_->Now() >= stop_episode_->deadline_ms) {
      FailStop(DiagnosticCode::kStopConfirmationTimeout,
               "backend did not confirm stop before deadline");
    }
    return;
  }
  if (!active_command_.has_value() || event.command_index != active_command_->command_index ||
      event.command_generation != active_command_->generation ||
      !active_command_->deadline_ms.has_value() || clock_->Now() < *active_command_->deadline_ms) {
    return;
  }
  const Command& command = program_.commands[active_command_->command_index];
  active_command_->status = CommandExecutionStatus::kTimedOut;
  DiagnosticCode code = DiagnosticCode::kMotionTimeout;
  if (command.type == CommandType::kWaitDi) {
    code = DiagnosticCode::kDiTimeout;
  } else if (command.type == CommandType::kSetTool) {
    code = DiagnosticCode::kToolTimeout;
  }
  AppendTrace(command, active_command_->command_index, CommandExecutionStatus::kTimedOut, state_,
              state_, RuntimeState::kFaulted, "command_timeout");
  BeginFault(
      code,
      "command timed out at deadline " + std::to_string(*active_command_->deadline_ms) + " ms",
      true, StopReason::kTimeout);
}

void RuntimeExecution::HandleMotionPoll(const RuntimeEvent& event) {
  if (!active_command_.has_value() || active_command_->generation != event.command_generation ||
      lifecycle_state_ != LifecycleState::kRunning) {
    return;
  }
  if (active_command_->type != CommandType::kMoveJ) {
    if (workcell_backend_ == nullptr) {
      return;
    }
    try {
      ApplyWorkcellSnapshot(workcell_backend_->PollWorkcell());
    } catch (const std::exception& error) {
      BeginFault(DiagnosticCode::kWorkcellSnapshotFailure, error.what(), false);
      return;
    }
    if (active_command_.has_value() && active_command_->deadline_ms.has_value() &&
        clock_->Now() < *active_command_->deadline_ms) {
      ScheduleMotionPoll(std::max<BusinessTime>(1, options_.default_stop_poll_ms),
                         active_command_->generation);
    }
    return;
  }
  MotionSnapshot snapshot = backend_->PollMotion();
  latest_positions_ = snapshot.positions;
  if (workcell_backend_ != nullptr) {
    try {
      ApplyWorkcellSnapshot(workcell_backend_->PollWorkcell());
    } catch (const std::exception& error) {
      BeginFault(DiagnosticCode::kWorkcellSnapshotFailure, error.what(), false);
      return;
    }
    if (!active_command_.has_value()) {
      return;
    }
  }
  const Command& command = program_.commands[active_command_->command_index];
  if (snapshot.status == MotionStatus::kRunning) {
    active_command_->status = CommandExecutionStatus::kRunning;
    AppendTrace(command, active_command_->command_index, CommandExecutionStatus::kRunning, state_,
                state_, state_, "motion_running", &snapshot);
    if (active_command_->deadline_ms.has_value() && clock_->Now() < *active_command_->deadline_ms) {
      ScheduleMotionPoll(
          std::max<BusinessTime>(1, snapshot.poll_after_ms.value_or(options_.default_stop_poll_ms)),
          active_command_->generation);
    }
    return;
  }
  if (snapshot.status == MotionStatus::kFailed) {
    active_command_->status = CommandExecutionStatus::kFailed;
    AppendTrace(command, active_command_->command_index, CommandExecutionStatus::kFailed, state_,
                state_, RuntimeState::kFaulted, "motion_failed", &snapshot);
    BeginFault(DiagnosticCode::kBackendFailure,
               snapshot.message.empty() ? "backend reported motion failure" : snapshot.message,
               true);
    return;
  }
  FinishActiveSuccess("motion_succeeded", &snapshot);
}

void RuntimeExecution::HandleDelayComplete(const RuntimeEvent& event) {
  if (active_command_.has_value() && active_command_->type == CommandType::kDelay &&
      active_command_->generation == event.command_generation &&
      lifecycle_state_ != LifecycleState::kPaused) {
    FinishActiveSuccess("delay_completed");
  }
}

void RuntimeExecution::PauseLocalCommand(const RuntimeEvent& event) {
  const LifecycleState before = lifecycle_state_;
  ActiveCommand& active = *active_command_;
  if (active.type == CommandType::kDelay) {
    active.remaining_ms = active.deadline_ms.has_value()
                              ? std::max<BusinessTime>(0, *active.deadline_ms - clock_->Now())
                              : 0;
  } else {
    active.remaining_ms = active.deadline_ms.has_value()
                              ? std::max<BusinessTime>(0, *active.deadline_ms - clock_->Now())
                              : 0;
  }
  active.deadline_ms.reset();
  active.status = CommandExecutionStatus::kPaused;
  ++active.generation;
  lifecycle_state_ = LifecycleState::kPaused;
  AppendControlTrace(event, ControlSubmissionStatus::kAccepted, "pause_accepted", before);
  const Command& command = program_.commands[active.command_index];
  AppendTrace(command, active.command_index, CommandExecutionStatus::kPaused, state_, state_,
              state_, "local_wait_paused");
}

void RuntimeExecution::ResumeActiveCommand() {
  if (!active_command_.has_value()) {
    lifecycle_state_ = LifecycleState::kRunning;
    return;
  }
  ActiveCommand& active = *active_command_;
  ++active.generation;
  if (active.type == CommandType::kMoveJ) {
    const Command& command = program_.commands[active.command_index];
    const std::optional<BusinessTime> deadline =
        CheckedBusinessTimeAdd(clock_->Now(), active.remaining_ms);
    if (!deadline.has_value()) {
      BeginFault(DiagnosticCode::kInvalidDuration, "resumed MOVEJ exceeds the business time range",
                 false);
      return;
    }
    const MotionSubmission submission = backend_->ResumeMoveJ(command.movej);
    if (submission.status == MotionSubmissionStatus::kRejected) {
      BeginFault(DiagnosticCode::kMotionRejected,
                 submission.message.empty() ? "backend rejected resumed MOVEJ" : submission.message,
                 true);
      return;
    }
    active.status = CommandExecutionStatus::kAccepted;
    active.started_at_ms = clock_->Now();
    active.deadline_ms = deadline;
    lifecycle_state_ = LifecycleState::kRunning;
    backend_operation_state_ = BackendOperationState::kActive;
    AppendTrace(command, active.command_index, CommandExecutionStatus::kAccepted, state_, state_,
                state_, "motion_resubmitted");
    ScheduleMotionPoll(submission.poll_after_ms.value_or(0), active.generation);
    ScheduleCommandTimeout(active);
    return;
  }
  lifecycle_state_ =
      workcell_backend_ == nullptr ? LifecycleState::kWaiting : LifecycleState::kRunning;
  active.status = CommandExecutionStatus::kAccepted;
  active.started_at_ms = clock_->Now();
  const Command& command = program_.commands[active.command_index];
  if (active.type == CommandType::kDelay) {
    active.deadline_ms = CheckedBusinessTimeAdd(clock_->Now(), active.remaining_ms);
    if (!active.deadline_ms.has_value()) {
      BeginFault(DiagnosticCode::kInvalidDuration, "resumed DELAY exceeds the business time range",
                 false);
      return;
    }
    events_.Schedule({*active.deadline_ms,
                      RuntimeEventPriority::kCommandCompletion,
                      RuntimeEventType::kDelayComplete,
                      {},
                      false,
                      {},
                      active.command_index,
                      active.generation});
    return;
  }
  active.deadline_ms = CheckedBusinessTimeAdd(clock_->Now(), active.remaining_ms);
  if (!active.deadline_ms.has_value()) {
    BeginFault(DiagnosticCode::kInvalidDuration, "resumed command exceeds the business time range",
               false);
    return;
  }
  if (workcell_backend_ == nullptr && active.type == CommandType::kWaitDi &&
      signals_.at(command.signal_name).value == command.signal_value) {
    FinishActiveSuccess("di_satisfied_while_paused");
    return;
  }
  if (workcell_backend_ == nullptr && active.type == CommandType::kSetTool &&
      tools_.at(command.tool_name).observed == command.tool_position) {
    FinishActiveSuccess("tool_satisfied_while_paused");
    return;
  }
  ScheduleCommandTimeout(active);
  if (workcell_backend_ != nullptr) {
    ScheduleMotionPoll(options_.default_stop_poll_ms, active.generation);
  }
}

void RuntimeExecution::ResetSession(const RuntimeEvent& event) {
  const LifecycleState before = lifecycle_state_;
  const bool recoverable_fault = lifecycle_state_ == LifecycleState::kFaulted &&
                                 result_.primary_alarm_index.has_value() &&
                                 result_.alarms[*result_.primary_alarm_index].recoverable;
  const bool allowed =
      (lifecycle_state_ == LifecycleState::kEmergencyStopped || recoverable_fault) &&
      !active_command_.has_value() && !stop_episode_.has_value() && servo_disabled_ &&
      !session_untrusted_ &&
      (workcell_backend_ == nullptr || (workcell_safe_requested_ && workcell_safe_confirmed_ &&
                                        workcell_safe_generation_.has_value()));
  if (!allowed) {
    AppendControlTrace(event, ControlSubmissionStatus::kRejected,
                       "reset_recovery_conditions_not_met", before);
    return;
  }
  if (workcell_backend_ != nullptr) {
    if (!connected_) {
      AppendControlTrace(event, ControlSubmissionStatus::kRejected,
                         "reset_workcell_connection_not_valid", before);
      return;
    }
    WorkcellCommandSubmission safe_state;
    WorkcellSnapshot snapshot;
    try {
      safe_state = workcell_backend_->PollSafeState();
      snapshot = workcell_backend_->PollWorkcell();
    } catch (const std::exception& error) {
      AppendControlTrace(event, ControlSubmissionStatus::kRejected,
                         "reset_workcell_revalidation_failed: " + std::string(error.what()),
                         before);
      return;
    }
    const bool scene_schema_matches =
        !result_.workcell.has_value() || snapshot.scene_schema == result_.workcell->scene_schema;
    if (safe_state.status == WorkcellCommandStatus::kRejected || !snapshot.valid ||
        !snapshot.simulation_running || snapshot.collision || !scene_schema_matches ||
        snapshot.generation <= *workcell_safe_generation_ ||
        (last_workcell_generation_.has_value() &&
         snapshot.generation <= *last_workcell_generation_)) {
      AppendControlTrace(event, ControlSubmissionStatus::kRejected,
                         "reset_workcell_revalidation_failed", before);
      return;
    }
    ApplyWorkcellSnapshot(snapshot);
    try {
      backend_->Disconnect();
      connected_ = false;
      backend_state_ = BackendState::kDisconnected;
    } catch (const std::exception& error) {
      session_untrusted_ = true;
      backend_state_ = BackendState::kFaulted;
      RaiseAlarm(DiagnosticCode::kDisconnectFailure, AlarmSeverity::kFault, "backend",
                 "disconnect_failed", error.what(), false,
                 {AlarmRecoveryCondition::kNewBackendSession});
      AppendControlTrace(event, ControlSubmissionStatus::kRejected, "reset_disconnect_failed",
                         before);
      return;
    }
  }
  for (Alarm& alarm : alarm_history_) {
    alarm.active = false;
  }
  std::vector<TraceEntry> previous_trace = std::move(result_.trace);
  result_ = {};
  result_.alarms = alarm_history_;
  result_.trace = std::move(previous_trace);
  events_.Clear();
  program_ = {};
  state_ = RuntimeState::kServoOff;
  lifecycle_state_ = LifecycleState::kIdle;
  backend_state_ = BackendState::kDisconnected;
  backend_operation_state_ = BackendOperationState::kIdle;
  task_terminal_ = false;
  workcell_safe_requested_ = false;
  workcell_safe_confirmed_ = false;
  workcell_safe_deadline_ms_.reset();
  workcell_safe_generation_.reset();
  last_workcell_generation_.reset();
  AppendControlTrace(event, ControlSubmissionStatus::kAccepted, "reset_accepted", before);
  SynchronizeResultState();
}

void RuntimeExecution::HandleControl(const RuntimeEvent& event) {
  const LifecycleState before = lifecycle_state_;
  switch (event.control_type) {
    case ControlType::kPause:
      if (lifecycle_state_ == LifecycleState::kPaused ||
          lifecycle_state_ == LifecycleState::kPausing) {
        AppendControlTrace(event, ControlSubmissionStatus::kIgnored, "pause_already_active",
                           before);
      } else if (lifecycle_state_ == LifecycleState::kRunning ||
                 lifecycle_state_ == LifecycleState::kWaiting) {
        if (!active_command_.has_value()) {
          lifecycle_state_ = LifecycleState::kPaused;
          AppendControlTrace(event, ControlSubmissionStatus::kAccepted, "pause_accepted", before);
        } else if (active_command_->type == CommandType::kMoveJ) {
          active_command_->remaining_ms =
              active_command_->deadline_ms.has_value()
                  ? std::max<BusinessTime>(0, *active_command_->deadline_ms - clock_->Now())
                  : options_.move_timeout_ms;
          active_command_->status = CommandExecutionStatus::kPausing;
          BeginStop(StopReason::kPause, StopDisposition::kPause, event);
        } else if (IsLocalWait(active_command_->type)) {
          PauseLocalCommand(event);
        } else {
          AppendControlTrace(event, ControlSubmissionStatus::kRejected,
                             "pause_requires_active_command", before);
        }
      } else {
        AppendControlTrace(event, ControlSubmissionStatus::kRejected, "pause_not_allowed", before);
      }
      break;
    case ControlType::kResume:
      if (lifecycle_state_ == LifecycleState::kPaused) {
        AppendControlTrace(event, ControlSubmissionStatus::kAccepted, "resume_accepted", before);
        ResumeActiveCommand();
      } else {
        AppendControlTrace(event, ControlSubmissionStatus::kIgnored, "resume_not_paused", before);
      }
      break;
    case ControlType::kStop:
      if (lifecycle_state_ == LifecycleState::kStopped ||
          lifecycle_state_ == LifecycleState::kIdle ||
          lifecycle_state_ == LifecycleState::kStopping ||
          lifecycle_state_ == LifecycleState::kFaulted) {
        AppendControlTrace(event, ControlSubmissionStatus::kIgnored, "stop_already_complete",
                           before);
      } else if (lifecycle_state_ == LifecycleState::kEmergencyStopping ||
                 lifecycle_state_ == LifecycleState::kEmergencyStopped) {
        AppendControlTrace(event, ControlSubmissionStatus::kIgnored, "stop_superseded_by_emergency",
                           before);
      } else if (active_command_.has_value() || connected_) {
        if (active_command_.has_value()) {
          active_command_->status = CommandExecutionStatus::kCancelling;
        }
        BeginStop(StopReason::kOperatorStop, StopDisposition::kOperatorStop, event);
      } else {
        AppendControlTrace(event, ControlSubmissionStatus::kAccepted, "stop_without_active_command",
                           before);
        FinishTerminal(ExecutionOutcome::kOperatorStopped);
      }
      break;
    case ControlType::kEstop:
      if (lifecycle_state_ == LifecycleState::kEmergencyStopping ||
          lifecycle_state_ == LifecycleState::kEmergencyStopped) {
        AppendControlTrace(event, ControlSubmissionStatus::kIgnored, "emergency_already_active",
                           before);
      } else if (active_command_.has_value() || connected_) {
        if (active_command_.has_value()) {
          active_command_->status = CommandExecutionStatus::kCancelling;
        }
        if (workcell_backend_ != nullptr && !workcell_safe_requested_) {
          workcell_safe_requested_ = true;
          workcell_safe_deadline_ms_ =
              SaturatingBusinessTimeAdd(clock_->Now(), options_.workcell_safe_state_timeout_ms);
          const WorkcellCommandSubmission safe_submission =
              workcell_backend_->RequestSafeState(StopReason::kEmergency);
          if (safe_submission.status == WorkcellCommandStatus::kRejected) {
            RaiseAlarm(DiagnosticCode::kBackendFailure, AlarmSeverity::kFault, "workcell",
                       "safe_state_rejected", safe_submission.message, false,
                       {AlarmRecoveryCondition::kNewBackendSession});
            session_untrusted_ = true;
          }
        }
        BeginStop(StopReason::kEmergency, StopDisposition::kEmergency, event);
      } else {
        AppendControlTrace(event, ControlSubmissionStatus::kAccepted, "emergency_without_backend",
                           before, StopReason::kEmergency, StopStatus::kStopped);
        RaiseAlarm(
            DiagnosticCode::kEmergencyStopped, AlarmSeverity::kEmergency, "runtime",
            "emergency_stop_confirmed", "emergency stop without an active backend", true,
            {AlarmRecoveryCondition::kNoActiveCommand, AlarmRecoveryCondition::kBackendStopped,
             AlarmRecoveryCondition::kServoDisabled});
        FinishTerminal(ExecutionOutcome::kEmergencyStopped);
      }
      break;
    case ControlType::kReset:
      ResetSession(event);
      break;
    case ControlType::kNone:
      AppendControlTrace(event, ControlSubmissionStatus::kRejected, "control_type_is_missing",
                         before);
      break;
  }
  SynchronizeResultState();
}

void RuntimeExecution::HandleNextCommand() {
  if (next_command_index_ >= program_.commands.size()) {
    BeginFault(DiagnosticCode::kInvalidRuntimeState, "program ended without a terminal command",
               false);
    return;
  }
  EnsureConnected();
  const std::size_t command_index = next_command_index_;
  const Command& command = program_.commands[command_index];
  const RuntimeState before = state_;
  switch (command.type) {
    case CommandType::kServoOn:
      backend_->SetServoEnabled(true);
      servo_disabled_ = false;
      state_ = RuntimeState::kReady;
      AppendTrace(command, command_index, CommandExecutionStatus::kSucceeded, before, before,
                  state_, "servo_enabled");
      ++next_command_index_;
      return;
    case CommandType::kServoOff:
      backend_->SetServoEnabled(false);
      servo_disabled_ = true;
      state_ = RuntimeState::kServoOff;
      AppendTrace(command, command_index, CommandExecutionStatus::kSucceeded, before, before,
                  state_, "servo_disabled");
      ++next_command_index_;
      return;
    case CommandType::kMoveJ: {
      const BusinessTime timeout = command.timeout_ms.value_or(options_.move_timeout_ms);
      const std::optional<BusinessTime> deadline = CheckedBusinessTimeAdd(clock_->Now(), timeout);
      if (!deadline.has_value()) {
        BeginFault(DiagnosticCode::kInvalidDuration,
                   "MOVEJ timeout exceeds the business time range", false);
        return;
      }
      ActiveCommand active{
          command_index, command.line, command.type, CommandExecutionStatus::kPending,
          clock_->Now(), deadline,     timeout,      1};
      active_command_ = active;
      const MotionSubmission submission = backend_->SubmitMoveJ(command.movej);
      if (submission.status == MotionSubmissionStatus::kRejected) {
        active_command_->status = CommandExecutionStatus::kFailed;
        AppendTrace(command, command_index, CommandExecutionStatus::kFailed, before, state_,
                    RuntimeState::kFaulted, "motion_rejected");
        BeginFault(DiagnosticCode::kMotionRejected,
                   submission.message.empty() ? "backend rejected MOVEJ" : submission.message,
                   true);
        return;
      }
      active_command_->status = CommandExecutionStatus::kAccepted;
      state_ = RuntimeState::kRunning;
      lifecycle_state_ = LifecycleState::kRunning;
      backend_operation_state_ = BackendOperationState::kActive;
      AppendTrace(command, command_index, CommandExecutionStatus::kAccepted, before, state_, state_,
                  "motion_accepted");
      ScheduleMotionPoll(submission.poll_after_ms.value_or(0), active_command_->generation);
      ScheduleCommandTimeout(*active_command_);
      return;
    }
    case CommandType::kDelay: {
      if (command.duration_ms == 0) {
        AppendTrace(command, command_index, CommandExecutionStatus::kSucceeded, before, before,
                    before, "delay_completed");
        ++next_command_index_;
        return;
      }
      const std::optional<BusinessTime> deadline =
          CheckedBusinessTimeAdd(clock_->Now(), command.duration_ms);
      if (!deadline.has_value()) {
        BeginFault(DiagnosticCode::kInvalidDuration,
                   "DELAY duration exceeds the business time range", false);
        return;
      }
      active_command_ = ActiveCommand{
          command_index, command.line, command.type,        CommandExecutionStatus::kAccepted,
          clock_->Now(), deadline,     command.duration_ms, 1};
      state_ = RuntimeState::kRunning;
      lifecycle_state_ = LifecycleState::kWaiting;
      AppendTrace(command, command_index, CommandExecutionStatus::kAccepted, before, state_, state_,
                  "delay_started");
      events_.Schedule({*deadline,
                        RuntimeEventPriority::kCommandCompletion,
                        RuntimeEventType::kDelayComplete,
                        {},
                        false,
                        {},
                        command_index,
                        active_command_->generation});
      return;
    }
    case CommandType::kSetDo:
      if (workcell_backend_ != nullptr) {
        const std::optional<BusinessTime> deadline =
            CheckedBusinessTimeAdd(clock_->Now(), options_.workcell_output_timeout_ms);
        if (!deadline.has_value()) {
          BeginFault(DiagnosticCode::kInvalidDuration,
                     "SET_DO timeout exceeds the business time range", false);
          return;
        }
        active_command_ = ActiveCommand{command_index,
                                        command.line,
                                        command.type,
                                        CommandExecutionStatus::kAccepted,
                                        clock_->Now(),
                                        deadline,
                                        options_.workcell_output_timeout_ms,
                                        1};
        const WorkcellCommandSubmission submission =
            workcell_backend_->SetDigitalOutput(command.signal_name, command.signal_value);
        if (submission.status == WorkcellCommandStatus::kRejected) {
          BeginFault(DiagnosticCode::kBackendFailure, submission.message, true);
          return;
        }
        state_ = RuntimeState::kRunning;
        lifecycle_state_ = LifecycleState::kRunning;
        backend_operation_state_ = BackendOperationState::kActive;
        AppendTrace(command, command_index, CommandExecutionStatus::kAccepted, before, state_,
                    state_, "workcell_do_accepted");
        ScheduleMotionPoll(submission.poll_after_ms.value_or(options_.default_stop_poll_ms),
                           active_command_->generation);
        ScheduleCommandTimeout(*active_command_);
        return;
      }
      signals_.at(command.signal_name).value = command.signal_value;
      signals_.at(command.signal_name).last_update_ms = clock_->Now();
      result_.signals = signals_;
      AppendTrace(command, command_index, CommandExecutionStatus::kSucceeded, before, before,
                  before, "do_updated");
      ++next_command_index_;
      return;
    case CommandType::kWaitDi:
    case CommandType::kSetTool: {
      if (command.type == CommandType::kSetTool && workcell_backend_ != nullptr) {
        const BusinessTime timeout = command.timeout_ms.value_or(
            command.tool_position == ToolPosition::kClosed ? options_.grip_timeout_ms
                                                           : options_.tool_timeout_ms);
        const std::optional<BusinessTime> deadline = CheckedBusinessTimeAdd(clock_->Now(), timeout);
        if (!deadline.has_value()) {
          BeginFault(DiagnosticCode::kInvalidDuration,
                     "SET_TOOL timeout exceeds the business time range", false);
          return;
        }
        const WorkcellCommandSubmission submission =
            workcell_backend_->SubmitToolCommand(command.tool_name, command.tool_position);
        if (submission.status == WorkcellCommandStatus::kRejected) {
          BeginFault(DiagnosticCode::kBackendFailure, submission.message, true);
          return;
        }
        tools_.at(command.tool_name).commanded = command.tool_position;
        result_.tools = tools_;
        active_command_ = ActiveCommand{
            command_index, command.line, command.type, CommandExecutionStatus::kAccepted,
            clock_->Now(), deadline,     timeout,      1};
        state_ = RuntimeState::kRunning;
        lifecycle_state_ = LifecycleState::kRunning;
        AppendTrace(command, command_index, CommandExecutionStatus::kAccepted, before, state_,
                    state_, "workcell_tool_accepted");
        ScheduleMotionPoll(submission.poll_after_ms.value_or(options_.default_stop_poll_ms),
                           active_command_->generation);
        ScheduleCommandTimeout(*active_command_);
        return;
      }
      if (command.type == CommandType::kSetTool) {
        tools_.at(command.tool_name).commanded = command.tool_position;
        result_.tools = tools_;
      }
      const BusinessTime timeout = command.timeout_ms.value_or(0);
      const std::optional<BusinessTime> deadline = CheckedBusinessTimeAdd(clock_->Now(), timeout);
      if (!deadline.has_value()) {
        BeginFault(DiagnosticCode::kInvalidDuration,
                   "local command timeout exceeds the business time range", false);
        return;
      }
      active_command_ = ActiveCommand{
          command_index, command.line, command.type, CommandExecutionStatus::kAccepted,
          clock_->Now(), deadline,     timeout,      1};
      state_ = RuntimeState::kRunning;
      lifecycle_state_ =
          workcell_backend_ == nullptr ? LifecycleState::kWaiting : LifecycleState::kRunning;
      AppendTrace(command, command_index, CommandExecutionStatus::kAccepted, before, state_, state_,
                  command.type == CommandType::kWaitDi ? "di_wait_started" : "tool_commanded");
      if ((command.type == CommandType::kWaitDi &&
           signals_.at(command.signal_name).value == command.signal_value) ||
          (command.type == CommandType::kSetTool &&
           tools_.at(command.tool_name).observed == command.tool_position)) {
        FinishActiveSuccess(command.type == CommandType::kWaitDi ? "di_already_satisfied"
                                                                 : "tool_already_satisfied");
      } else {
        ScheduleCommandTimeout(*active_command_);
        if (workcell_backend_ != nullptr) {
          ScheduleMotionPoll(options_.default_stop_poll_ms, active_command_->generation);
        }
      }
      return;
    }
    case CommandType::kStop:
      active_command_ = ActiveCommand{command_index,
                                      command.line,
                                      command.type,
                                      CommandExecutionStatus::kCancelling,
                                      clock_->Now(),
                                      std::nullopt,
                                      0,
                                      1};
      BeginStop(StopReason::kProgramComplete, StopDisposition::kProgramComplete);
      return;
    case CommandType::kNone:
      BeginFault(DiagnosticCode::kTaskSyntax, "command type is missing", false);
      return;
  }
}

void RuntimeExecution::HandleEvent(const RuntimeEvent& event) {
  switch (event.type) {
    case RuntimeEventType::kControl:
      HandleControl(event);
      break;
    case RuntimeEventType::kMotionPoll:
      HandleMotionPoll(event);
      break;
    case RuntimeEventType::kDelayComplete:
      HandleDelayComplete(event);
      break;
    case RuntimeEventType::kStopPoll:
      HandleStopPoll(event);
      break;
    case RuntimeEventType::kDigitalInput:
    case RuntimeEventType::kToolFeedback:
    case RuntimeEventType::kBackendFault:
      if (event.type == RuntimeEventType::kBackendFault &&
          (event.detail == "timeout" || event.detail == "stop_timeout")) {
        HandleTimeout(event);
      } else {
        HandleExternalEvent(event);
      }
      break;
  }
}

ExecutionAdvanceResult RuntimeExecution::Advance() {
  const auto finish = [this](ExecutionAdvanceResult result) {
    RefreshSnapshot();
    return result;
  };
  try {
    const std::optional<RuntimeEvent> next = events_.PeekNext();
    const bool must_process_event =
        next.has_value() &&
        (next->time <= clock_->Now() || active_command_.has_value() || stop_episode_.has_value() ||
         lifecycle_state_ == LifecycleState::kPaused ||
         (task_terminal_ && next->type == RuntimeEventType::kControl));
    if (must_process_event) {
      const RuntimeEvent event = *events_.PopNext();
      AdvanceClockTo(event.time);
      HandleEvent(event);
      return finish(task_terminal_ && !events_.PeekNext().has_value()
                        ? ExecutionAdvanceResult::kTerminal
                        : ExecutionAdvanceResult::kProgressed);
    }
    if (task_terminal_) {
      return finish(ExecutionAdvanceResult::kTerminal);
    }
    if (lifecycle_state_ == LifecycleState::kIdle) {
      return finish(ExecutionAdvanceResult::kIdle);
    }
    if (lifecycle_state_ == LifecycleState::kPaused) {
      return finish(ExecutionAdvanceResult::kProgressed);
    }
    HandleNextCommand();
  } catch (const std::exception& error) {
    BeginFault(DiagnosticCode::kBackendFailure, error.what(), false);
  }
  return finish(task_terminal_ ? ExecutionAdvanceResult::kTerminal
                               : ExecutionAdvanceResult::kProgressed);
}

RuntimeExecution RobotTaskRuntime::Start(const Program& program, RobotBackend* backend,
                                         Clock* clock) const {
  return Start(program, nullptr, nullptr, nullptr, nullptr, nullptr, backend, clock);
}

RuntimeExecution RobotTaskRuntime::Start(const Program& program,
                                         const RobotConfiguration* robot_configuration,
                                         const PointCatalog* point_catalog, RobotBackend* backend,
                                         Clock* clock) const {
  return Start(program, robot_configuration, point_catalog, nullptr, nullptr, nullptr, backend,
               clock);
}

RuntimeExecution RobotTaskRuntime::Start(const Program& program,
                                         const RobotConfiguration* robot_configuration,
                                         const PointCatalog* point_catalog,
                                         const IOConfiguration* io_configuration,
                                         const ToolConfiguration* tool_configuration,
                                         const MockScenario* scenario, RobotBackend* backend,
                                         Clock* clock, WorkcellBackend* workcell_backend) const {
  RuntimeExecution execution(backend, clock, robot_configuration, point_catalog, io_configuration,
                             tool_configuration, scenario, {}, workcell_backend);
  execution.SubmitProgram(program);
  return execution;
}

ExecutionResult RobotTaskRuntime::Execute(const Program& program, RobotBackend* backend) const {
  return Execute(program, nullptr, nullptr, nullptr, nullptr, nullptr, backend);
}

ExecutionResult RobotTaskRuntime::Execute(const Program& program,
                                          const RobotConfiguration* robot_configuration,
                                          const PointCatalog* point_catalog,
                                          RobotBackend* backend) const {
  return Execute(program, robot_configuration, point_catalog, nullptr, nullptr, nullptr, backend);
}

ExecutionResult RobotTaskRuntime::Execute(const Program& program,
                                          const RobotConfiguration* robot_configuration,
                                          const PointCatalog* point_catalog,
                                          const IOConfiguration* io_configuration,
                                          const ToolConfiguration* tool_configuration,
                                          const MockScenario* scenario, RobotBackend* backend,
                                          WorkcellBackend* workcell_backend) const {
  VirtualClock clock;
  RuntimeExecution execution =
      Start(program, robot_configuration, point_catalog, io_configuration, tool_configuration,
            scenario, backend, &clock, workcell_backend);
  for (int step = 0; !execution.is_terminal() && step < 100000; ++step) {
    execution.Advance();
  }
  if (!execution.is_terminal()) {
    execution.SubmitControl(ControlType::kEstop, "one-shot-guard");
    while (!execution.is_terminal()) {
      execution.Advance();
    }
  }
  return execution.result();
}

}  // namespace roborun
