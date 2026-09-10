#include "roborun/types.h"

#include <iomanip>
#include <sstream>

namespace roborun {

std::string ToString(CommandType type) {
  switch (type) {
    case CommandType::kServoOn:
      return "SERVO_ON";
    case CommandType::kServoOff:
      return "SERVO_OFF";
    case CommandType::kMoveJ:
      return "MOVEJ";
    case CommandType::kDelay:
      return "DELAY";
    case CommandType::kSetDo:
      return "SET_DO";
    case CommandType::kWaitDi:
      return "WAIT_DI";
    case CommandType::kSetTool:
      return "SET_TOOL";
    case CommandType::kStop:
      return "STOP";
    case CommandType::kNone:
      return "NONE";
  }
  return "UNKNOWN";
}

std::string ToString(SignalDirection direction) {
  return direction == SignalDirection::kDi ? "DI" : "DO";
}

std::string ToString(ToolType type) { return type == ToolType::kGripper ? "gripper" : "unknown"; }

std::string ToString(ToolPosition position) {
  return position == ToolPosition::kOpen ? "OPEN" : "CLOSED";
}

std::string ToString(RuntimeState state) {
  switch (state) {
    case RuntimeState::kServoOff:
      return "servo_off";
    case RuntimeState::kReady:
      return "ready";
    case RuntimeState::kRunning:
      return "running";
    case RuntimeState::kStopped:
      return "stopped";
    case RuntimeState::kFaulted:
      return "faulted";
  }
  return "unknown";
}

std::string ToString(ValidatorServoState state) {
  switch (state) {
    case ValidatorServoState::kServoOff:
      return "servo_off";
    case ValidatorServoState::kReady:
      return "ready";
    case ValidatorServoState::kStopped:
      return "stopped";
  }
  return "unknown";
}

std::string ToString(LifecycleState state) {
  switch (state) {
    case LifecycleState::kIdle:
      return "idle";
    case LifecycleState::kRunning:
      return "running";
    case LifecycleState::kWaiting:
      return "waiting";
    case LifecycleState::kPausing:
      return "pausing";
    case LifecycleState::kPaused:
      return "paused";
    case LifecycleState::kStopping:
      return "stopping";
    case LifecycleState::kStopped:
      return "stopped";
    case LifecycleState::kEmergencyStopping:
      return "emergency_stopping";
    case LifecycleState::kEmergencyStopped:
      return "emergency_stopped";
    case LifecycleState::kFaulted:
      return "faulted";
  }
  return "unknown";
}

std::string ToString(ControlType type) {
  switch (type) {
    case ControlType::kNone:
      return "none";
    case ControlType::kPause:
      return "pause";
    case ControlType::kResume:
      return "resume";
    case ControlType::kStop:
      return "stop";
    case ControlType::kEstop:
      return "estop";
    case ControlType::kReset:
      return "reset";
  }
  return "unknown";
}

std::string ToString(ControlSubmissionStatus status) {
  switch (status) {
    case ControlSubmissionStatus::kNone:
      return "none";
    case ControlSubmissionStatus::kQueued:
      return "queued";
    case ControlSubmissionStatus::kAccepted:
      return "accepted";
    case ControlSubmissionStatus::kIgnored:
      return "ignored";
    case ControlSubmissionStatus::kRejected:
      return "rejected";
  }
  return "unknown";
}

std::string ToString(ExecutionOutcome outcome) {
  switch (outcome) {
    case ExecutionOutcome::kNone:
      return "none";
    case ExecutionOutcome::kProgramCompleted:
      return "program_completed";
    case ExecutionOutcome::kOperatorStopped:
      return "operator_stopped";
    case ExecutionOutcome::kEmergencyStopped:
      return "emergency_stopped";
    case ExecutionOutcome::kFaulted:
      return "faulted";
  }
  return "unknown";
}

std::string ToString(ProgramSubmissionStatus status) {
  return status == ProgramSubmissionStatus::kAccepted ? "accepted" : "rejected";
}

std::string ToString(CommandExecutionStatus status) {
  switch (status) {
    case CommandExecutionStatus::kPending:
      return "pending";
    case CommandExecutionStatus::kAccepted:
      return "accepted";
    case CommandExecutionStatus::kRunning:
      return "running";
    case CommandExecutionStatus::kPausing:
      return "pausing";
    case CommandExecutionStatus::kPaused:
      return "paused";
    case CommandExecutionStatus::kCancelling:
      return "cancelling";
    case CommandExecutionStatus::kCancelled:
      return "cancelled";
    case CommandExecutionStatus::kSucceeded:
      return "succeeded";
    case CommandExecutionStatus::kTimedOut:
      return "timed_out";
    case CommandExecutionStatus::kFailed:
      return "failed";
  }
  return "unknown";
}

std::string ToString(BackendState state) {
  switch (state) {
    case BackendState::kDisconnected:
      return "disconnected";
    case BackendState::kConnected:
      return "connected";
    case BackendState::kFaulted:
      return "faulted";
  }
  return "unknown";
}

std::string ToString(BackendOperationState state) {
  switch (state) {
    case BackendOperationState::kIdle:
      return "idle";
    case BackendOperationState::kActive:
      return "active";
    case BackendOperationState::kStopping:
      return "stopping";
    case BackendOperationState::kStopped:
      return "stopped";
  }
  return "unknown";
}

std::string ToString(StopReason reason) {
  switch (reason) {
    case StopReason::kProgramComplete:
      return "program_complete";
    case StopReason::kPause:
      return "pause";
    case StopReason::kOperatorStop:
      return "operator_stop";
    case StopReason::kEmergency:
      return "emergency";
    case StopReason::kTimeout:
      return "timeout";
    case StopReason::kFaultCleanup:
      return "fault_cleanup";
  }
  return "unknown";
}

std::string ToString(StopStatus status) {
  switch (status) {
    case StopStatus::kStopping:
      return "stopping";
    case StopStatus::kStopped:
      return "stopped";
    case StopStatus::kFailed:
      return "failed";
  }
  return "unknown";
}

std::string ToString(AlarmSeverity severity) {
  return severity == AlarmSeverity::kEmergency ? "emergency" : "fault";
}

std::string ToString(AlarmRecoveryCondition condition) {
  switch (condition) {
    case AlarmRecoveryCondition::kNoActiveCommand:
      return "no_active_command";
    case AlarmRecoveryCondition::kBackendStopped:
      return "backend_stopped";
    case AlarmRecoveryCondition::kServoDisabled:
      return "servo_disabled";
    case AlarmRecoveryCondition::kNewBackendSession:
      return "new_backend_session";
  }
  return "unknown";
}

std::string ToString(DiagnosticCode code) {
  switch (code) {
    case DiagnosticCode::kConfigIo:
      return "CONFIG_IO";
    case DiagnosticCode::kJsonSyntax:
      return "JSON_SYNTAX";
    case DiagnosticCode::kSchemaMismatch:
      return "SCHEMA_MISMATCH";
    case DiagnosticCode::kInvalidConfiguration:
      return "INVALID_CONFIGURATION";
    case DiagnosticCode::kRobotMismatch:
      return "ROBOT_MISMATCH";
    case DiagnosticCode::kDuplicatePoint:
      return "DUPLICATE_POINT";
    case DiagnosticCode::kInvalidPointName:
      return "INVALID_POINT_NAME";
    case DiagnosticCode::kInvalidJointCount:
      return "INVALID_JOINT_COUNT";
    case DiagnosticCode::kNonFiniteJoint:
      return "NONFINITE_JOINT";
    case DiagnosticCode::kJointLimitViolation:
      return "JOINT_LIMIT_VIOLATION";
    case DiagnosticCode::kUnknownPoint:
      return "UNKNOWN_POINT";
    case DiagnosticCode::kInvalidSpeed:
      return "INVALID_SPEED";
    case DiagnosticCode::kInvalidRuntimeState:
      return "INVALID_RUNTIME_STATE";
    case DiagnosticCode::kTaskSyntax:
      return "TASK_SYNTAX";
    case DiagnosticCode::kInvalidDuration:
      return "INVALID_DURATION";
    case DiagnosticCode::kUnknownSignal:
      return "UNKNOWN_SIGNAL";
    case DiagnosticCode::kSignalDirectionMismatch:
      return "SIGNAL_DIRECTION_MISMATCH";
    case DiagnosticCode::kDuplicateSignal:
      return "DUPLICATE_SIGNAL";
    case DiagnosticCode::kInvalidSignalConfiguration:
      return "INVALID_SIGNAL_CONFIGURATION";
    case DiagnosticCode::kUnknownTool:
      return "UNKNOWN_TOOL";
    case DiagnosticCode::kInvalidToolConfiguration:
      return "INVALID_TOOL_CONFIGURATION";
    case DiagnosticCode::kInvalidToolState:
      return "INVALID_TOOL_STATE";
    case DiagnosticCode::kInvalidScenario:
      return "INVALID_SCENARIO";
    case DiagnosticCode::kMotionRejected:
      return "MOTION_REJECTED";
    case DiagnosticCode::kMotionTimeout:
      return "MOTION_TIMEOUT";
    case DiagnosticCode::kDiTimeout:
      return "DI_TIMEOUT";
    case DiagnosticCode::kToolTimeout:
      return "TOOL_TIMEOUT";
    case DiagnosticCode::kBackendFailure:
      return "BACKEND_FAILURE";
    case DiagnosticCode::kBackendConnectionLoss:
      return "BACKEND_CONNECTION_LOSS";
    case DiagnosticCode::kStopRejected:
      return "STOP_REJECTED";
    case DiagnosticCode::kStopConfirmationTimeout:
      return "STOP_CONFIRMATION_TIMEOUT";
    case DiagnosticCode::kStopPollingFailure:
      return "STOP_POLLING_FAILURE";
    case DiagnosticCode::kServoDisableFailure:
      return "SERVO_DISABLE_FAILURE";
    case DiagnosticCode::kDisconnectFailure:
      return "DISCONNECT_FAILURE";
    case DiagnosticCode::kEmergencyStopped:
      return "EMERGENCY_STOPPED";
    case DiagnosticCode::kInvalidWorkcellConfiguration:
      return "INVALID_WORKCELL_CONFIGURATION";
    case DiagnosticCode::kDuplicateWorkcellAlias:
      return "DUPLICATE_WORKCELL_ALIAS";
    case DiagnosticCode::kWorkcellSchemaMismatch:
      return "WORKCELL_SCHEMA_MISMATCH";
    case DiagnosticCode::kWorkcellSnapshotFailure:
      return "WORKCELL_SNAPSHOT_FAILURE";
    case DiagnosticCode::kGripFailure:
      return "GRIP_FAILURE";
    case DiagnosticCode::kPlacementFailure:
      return "PLACEMENT_FAILURE";
    case DiagnosticCode::kCollisionDetected:
      return "COLLISION_DETECTED";
  }
  return "UNKNOWN";
}

std::string FormatJointPositions(const JointPositions& positions) {
  std::ostringstream stream;
  stream << '[' << std::fixed << std::setprecision(4);
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << positions[index];
  }
  stream << ']';
  return stream.str();
}

}  // namespace roborun
