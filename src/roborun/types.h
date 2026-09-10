#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace roborun {

constexpr std::size_t kJointCount = 6;
using JointPositions = std::array<double, kJointCount>;
using Pose = std::array<double, 7>;
using BusinessTime = std::int64_t;

enum class CommandType {
  kServoOn,
  kServoOff,
  kMoveJ,
  kDelay,
  kSetDo,
  kWaitDi,
  kSetTool,
  kStop,
  kNone,
};

enum class RuntimeState {
  kServoOff,
  kReady,
  kRunning,
  kStopped,
  kFaulted,
};

// Static validation deliberately uses a separate state from the Runtime Session.
enum class ValidatorServoState {
  kServoOff,
  kReady,
  kStopped,
};

enum class LifecycleState {
  kIdle,
  kRunning,
  kWaiting,
  kPausing,
  kPaused,
  kStopping,
  kStopped,
  kEmergencyStopping,
  kEmergencyStopped,
  kFaulted,
};

enum class ControlType {
  kNone,
  kPause,
  kResume,
  kStop,
  kEstop,
  kReset,
};

enum class ControlSubmissionStatus {
  kNone,
  kQueued,
  kAccepted,
  kIgnored,
  kRejected,
};

enum class ExecutionOutcome {
  kNone,
  kProgramCompleted,
  kOperatorStopped,
  kEmergencyStopped,
  kFaulted,
};

enum class ProgramSubmissionStatus {
  kAccepted,
  kRejected,
};

enum class SignalDirection {
  kDi,
  kDo,
};

enum class ToolType {
  kGripper,
};

enum class ToolPosition {
  kOpen,
  kClosed,
};

enum class CommandExecutionStatus {
  kPending,
  kAccepted,
  kRunning,
  kPausing,
  kPaused,
  kCancelling,
  kCancelled,
  kSucceeded,
  kTimedOut,
  kFailed,
};

enum class BackendState {
  kDisconnected,
  kConnected,
  kFaulted,
};

enum class BackendOperationState {
  kIdle,
  kActive,
  kStopping,
  kStopped,
};

enum class StopReason {
  kProgramComplete,
  kPause,
  kOperatorStop,
  kEmergency,
  kTimeout,
  kFaultCleanup,
};

enum class StopStatus {
  kStopping,
  kStopped,
  kFailed,
};

enum class AlarmSeverity {
  kFault,
  kEmergency,
};

enum class AlarmRecoveryCondition {
  kNoActiveCommand,
  kBackendStopped,
  kServoDisabled,
  kNewBackendSession,
};

enum class DiagnosticCode {
  kConfigIo,
  kJsonSyntax,
  kSchemaMismatch,
  kInvalidConfiguration,
  kRobotMismatch,
  kDuplicatePoint,
  kInvalidPointName,
  kInvalidJointCount,
  kNonFiniteJoint,
  kJointLimitViolation,
  kUnknownPoint,
  kInvalidSpeed,
  kInvalidRuntimeState,
  kTaskSyntax,
  kInvalidDuration,
  kUnknownSignal,
  kSignalDirectionMismatch,
  kDuplicateSignal,
  kInvalidSignalConfiguration,
  kUnknownTool,
  kInvalidToolConfiguration,
  kInvalidToolState,
  kInvalidScenario,
  kMotionRejected,
  kMotionTimeout,
  kDiTimeout,
  kToolTimeout,
  kBackendFailure,
  kBackendConnectionLoss,
  kStopRejected,
  kStopConfirmationTimeout,
  kStopPollingFailure,
  kServoDisableFailure,
  kDisconnectFailure,
  kEmergencyStopped,
  kInvalidWorkcellConfiguration,
  kDuplicateWorkcellAlias,
  kWorkcellSchemaMismatch,
  kWorkcellSnapshotFailure,
  kGripFailure,
  kPlacementFailure,
  kCollisionDetected,
};

struct Diagnostic {
  DiagnosticCode code = DiagnosticCode::kTaskSyntax;
  std::string source;
  std::size_t line = 0;
  std::size_t column = 0;
  std::string path;
  std::string message;
};

struct Alarm {
  DiagnosticCode code = DiagnosticCode::kBackendFailure;
  AlarmSeverity severity = AlarmSeverity::kFault;
  std::string source;
  std::string reason;
  std::string details;
  BusinessTime raised_business_time_ms = 0;
  bool recoverable = false;
  bool active = true;
  std::vector<AlarmRecoveryCondition> recovery_conditions;
};

struct MoveJRequest {
  JointPositions targets{};
  int speed_percent = 100;
  double joint_speed_radians_per_second = 1.0;
  std::string point_name;
};

struct DigitalSignal {
  std::string name;
  SignalDirection direction = SignalDirection::kDi;
  bool value = false;
  BusinessTime last_update_ms = 0;
};

struct ToolState {
  std::string name;
  ToolType type = ToolType::kGripper;
  ToolPosition commanded = ToolPosition::kOpen;
  ToolPosition observed = ToolPosition::kOpen;
  BusinessTime last_feedback_ms = 0;
};

struct Command {
  CommandType type = CommandType::kNone;
  std::size_t line = 0;
  std::size_t column = 1;
  MoveJRequest movej;
  std::string point_reference;
  BusinessTime duration_ms = 0;
  std::optional<BusinessTime> timeout_ms;
  std::string signal_name;
  bool signal_value = false;
  std::string tool_name;
  ToolPosition tool_position = ToolPosition::kOpen;
};

struct ActiveCommand {
  std::size_t command_index = 0;
  std::size_t line = 0;
  CommandType type = CommandType::kNone;
  CommandExecutionStatus status = CommandExecutionStatus::kPending;
  BusinessTime started_at_ms = 0;
  std::optional<BusinessTime> deadline_ms;
  BusinessTime remaining_ms = 0;
  std::uint64_t generation = 1;
};

struct Program {
  std::string source;
  std::vector<Command> commands;
};

struct ParseResult {
  Program program;
  std::vector<Diagnostic> diagnostics;
};

struct ValidationResult {
  Program program;
  std::vector<Diagnostic> diagnostics;
};

struct TraceEntry {
  BusinessTime business_time_ms = 0;
  std::size_t command_index = 0;
  std::size_t line = 0;
  CommandType command = CommandType::kNone;
  RuntimeState state_before = RuntimeState::kServoOff;
  RuntimeState state_during = RuntimeState::kServoOff;
  RuntimeState state_after = RuntimeState::kServoOff;
  LifecycleState lifecycle_before = LifecycleState::kIdle;
  LifecycleState lifecycle_during = LifecycleState::kIdle;
  LifecycleState lifecycle_after = LifecycleState::kIdle;
  CommandExecutionStatus command_status = CommandExecutionStatus::kPending;
  ControlType control_type = ControlType::kNone;
  ControlSubmissionStatus control_status = ControlSubmissionStatus::kNone;
  std::uint64_t control_request_id = 0;
  std::string control_source;
  StopReason stop_reason = StopReason::kProgramComplete;
  StopStatus stop_status = StopStatus::kStopped;
  ExecutionOutcome outcome = ExecutionOutcome::kNone;
  std::string event_cause;
  std::string point_name;
  std::string signal_name;
  std::string tool_name;
  bool expected_value = false;
  bool observed_value = false;
  BusinessTime deadline_ms = -1;
  ToolPosition commanded_tool_position = ToolPosition::kOpen;
  ToolPosition observed_tool_position = ToolPosition::kOpen;
  int speed_percent = 0;
  int backend_steps = 0;
  std::uint64_t workcell_generation = 0;
  JointPositions targets{};
  JointPositions actual_positions{};
};

struct WorkcellEvidence {
  std::uint64_t generation = 0;
  std::string scene_schema;
  JointPositions joint_positions{};
  std::map<std::string, DigitalSignal> signals;
  std::map<std::string, ToolState> tools;
  std::map<std::string, double> tool_openings;
  bool workpiece_present = false;
  bool workpiece_attached = false;
  bool workpiece_at_place = false;
  Pose workpiece_pose{};
  bool collision = false;
  std::vector<std::string> collision_aliases;
  bool simulation_running = false;
};

struct ExecutionResult {
  bool succeeded = false;
  ExecutionOutcome outcome = ExecutionOutcome::kNone;
  RuntimeState final_state = RuntimeState::kServoOff;
  LifecycleState lifecycle_state = LifecycleState::kIdle;
  BackendState backend_state = BackendState::kDisconnected;
  BackendOperationState backend_operation_state = BackendOperationState::kIdle;
  std::string task_source;
  std::string backend_name;
  std::string backend_version;
  std::string started_at;
  std::string finished_at;
  BusinessTime finished_business_time_ms = 0;
  std::map<std::string, DigitalSignal> signals;
  std::map<std::string, ToolState> tools;
  std::optional<WorkcellEvidence> workcell;
  bool workcell_safe_confirmed = false;
  std::vector<TraceEntry> trace;
  std::vector<Diagnostic> diagnostics;
  std::vector<Alarm> alarms;
  std::optional<std::size_t> primary_alarm_index;
};

struct ProgramSubmission {
  ProgramSubmissionStatus status = ProgramSubmissionStatus::kRejected;
  std::string message;
};

struct ControlSubmission {
  std::uint64_t request_id = 0;
  ControlType type = ControlType::kNone;
  ControlSubmissionStatus status = ControlSubmissionStatus::kRejected;
  std::string message;
};

// A value-only observation of a RuntimeSession.  External adapters can retain
// this object without holding references into the Runtime or advancing it.
struct RuntimeTerminalSummary {
  bool succeeded = false;
  ExecutionOutcome outcome = ExecutionOutcome::kNone;
  RuntimeState final_state = RuntimeState::kServoOff;
  BusinessTime finished_business_time_ms = 0;
  std::vector<Diagnostic> diagnostics;
};

struct RuntimeSnapshot {
  std::uint64_t sequence = 0;
  RuntimeState runtime_state = RuntimeState::kServoOff;
  LifecycleState lifecycle_state = LifecycleState::kIdle;
  BackendState backend_state = BackendState::kDisconnected;
  BackendOperationState backend_operation_state = BackendOperationState::kIdle;
  std::optional<ActiveCommand> active_command;
  BusinessTime business_time_ms = 0;
  JointPositions joint_positions{};
  std::map<std::string, DigitalSignal> signals;
  std::map<std::string, ToolState> tools;
  std::vector<Alarm> alarms;
  std::optional<std::size_t> primary_alarm_index;
  std::optional<RuntimeTerminalSummary> terminal;
};

std::string ToString(CommandType type);
std::string ToString(RuntimeState state);
std::string ToString(ValidatorServoState state);
std::string ToString(LifecycleState state);
std::string ToString(ControlType type);
std::string ToString(ControlSubmissionStatus status);
std::string ToString(ExecutionOutcome outcome);
std::string ToString(ProgramSubmissionStatus status);
std::string ToString(SignalDirection direction);
std::string ToString(ToolType type);
std::string ToString(ToolPosition position);
std::string ToString(CommandExecutionStatus status);
std::string ToString(BackendState state);
std::string ToString(BackendOperationState state);
std::string ToString(StopReason reason);
std::string ToString(StopStatus status);
std::string ToString(AlarmSeverity severity);
std::string ToString(AlarmRecoveryCondition condition);
std::string ToString(DiagnosticCode code);
std::string FormatJointPositions(const JointPositions& positions);

}  // namespace roborun
