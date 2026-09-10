#include "roborun/validator.h"

#include <cmath>

namespace roborun {
namespace {

constexpr double kJointLimitRadians = 2.0 * 3.14159265358979323846;

void AddDiagnostic(ValidationResult* result, const Program& program, std::size_t line,
                   DiagnosticCode code, const std::string& message) {
  result->diagnostics.push_back(Diagnostic{code, program.source, line, 1, "", message});
}

}  // namespace

ValidationResult ValidateProgram(const Program& program,
                                 const RobotConfiguration* robot_configuration,
                                 const PointCatalog* point_catalog,
                                 const IOConfiguration* io_configuration,
                                 const ToolConfiguration* tool_configuration) {
  ValidationResult result;
  result.program = program;
  ValidatorServoState state = ValidatorServoState::kServoOff;

  if (program.commands.empty()) {
    AddDiagnostic(&result, program, 0, DiagnosticCode::kTaskSyntax,
                  "program does not contain any commands");
    return result;
  }

  for (std::size_t command_index = 0; command_index < program.commands.size(); ++command_index) {
    const Command& command = result.program.commands[command_index];
    if (state == ValidatorServoState::kStopped) {
      AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                    "command appears after STOP");
      continue;
    }

    switch (command.type) {
      case CommandType::kServoOn:
        if (state != ValidatorServoState::kServoOff) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                        "SERVO_ON requires servo_off state");
          continue;
        }
        state = ValidatorServoState::kReady;
        break;

      case CommandType::kServoOff:
        if (state != ValidatorServoState::kReady) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                        "SERVO_OFF requires ready state");
          continue;
        }
        state = ValidatorServoState::kServoOff;
        break;

      case CommandType::kMoveJ: {
        if (state != ValidatorServoState::kReady) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                        "MOVEJ requires a preceding SERVO_ON command");
          continue;
        }
        if (command.movej.speed_percent < 1 || command.movej.speed_percent > 100) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidSpeed,
                        "SPEED must be in the range 1 to 100");
        }
        const double reference_joint_speed =
            robot_configuration == nullptr ? 1.0 : robot_configuration->reference_joint_speed;
        result.program.commands[command_index].movej.joint_speed_radians_per_second =
            reference_joint_speed * command.movej.speed_percent / 100.0;
        if (!command.point_reference.empty()) {
          if (robot_configuration == nullptr) {
            AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidConfiguration,
                          "named MOVEJ requires a robot configuration");
            continue;
          }
          if (point_catalog == nullptr) {
            AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidConfiguration,
                          "named MOVEJ requires a point catalog");
            continue;
          }
          const auto point = point_catalog->points.find(command.point_reference);
          if (point == point_catalog->points.end()) {
            AddDiagnostic(&result, program, command.line, DiagnosticCode::kUnknownPoint,
                          "unknown point: " + command.point_reference);
            continue;
          }
          result.program.commands[command_index].movej.targets = point->second.joints;
          result.program.commands[command_index].movej.point_name = command.point_reference;
        }
        for (std::size_t index = 0; index < kJointCount; ++index) {
          const double target = result.program.commands[command_index].movej.targets[index];
          const double minimum = robot_configuration == nullptr
                                     ? -kJointLimitRadians
                                     : robot_configuration->joints[index].minimum;
          const double maximum = robot_configuration == nullptr
                                     ? kJointLimitRadians
                                     : robot_configuration->joints[index].maximum;
          if (!std::isfinite(target)) {
            AddDiagnostic(&result, program, command.line, DiagnosticCode::kNonFiniteJoint,
                          "MOVEJ target " + std::to_string(index + 1) + " must be finite");
          } else if (target < minimum || target > maximum) {
            AddDiagnostic(&result, program, command.line, DiagnosticCode::kJointLimitViolation,
                          "MOVEJ target " + std::to_string(index + 1) +
                              " is outside the configured joint limit");
          }
        }
        break;
      }

      case CommandType::kDelay:
        if (state != ValidatorServoState::kReady) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                        "DELAY requires a ready servo state");
        }
        break;

      case CommandType::kSetDo:
        if (state != ValidatorServoState::kReady) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                        "SET_DO requires a ready servo state");
        }
        if (io_configuration == nullptr) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidConfiguration,
                        "SET_DO requires an I/O configuration");
          break;
        }
        if (!io_configuration->signals.contains(command.signal_name)) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kUnknownSignal,
                        "unknown signal: " + command.signal_name);
        } else if (io_configuration->signals.at(command.signal_name).direction !=
                   SignalDirection::kDo) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kSignalDirectionMismatch,
                        "SET_DO requires a DO signal: " + command.signal_name);
        }
        break;

      case CommandType::kWaitDi:
        if (state != ValidatorServoState::kReady) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                        "WAIT_DI requires a ready servo state");
        }
        if (io_configuration == nullptr) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidConfiguration,
                        "WAIT_DI requires an I/O configuration");
          break;
        }
        if (!io_configuration->signals.contains(command.signal_name)) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kUnknownSignal,
                        "unknown signal: " + command.signal_name);
        } else if (io_configuration->signals.at(command.signal_name).direction !=
                   SignalDirection::kDi) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kSignalDirectionMismatch,
                        "WAIT_DI requires a DI signal: " + command.signal_name);
        }
        break;

      case CommandType::kSetTool:
        if (state != ValidatorServoState::kReady) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                        "SET_TOOL requires a ready servo state");
        }
        if (tool_configuration == nullptr) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidConfiguration,
                        "SET_TOOL requires a tool configuration");
          break;
        }
        if (!tool_configuration->tools.contains(command.tool_name)) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kUnknownTool,
                        "unknown tool: " + command.tool_name);
        }
        break;

      case CommandType::kStop:
        if (state != ValidatorServoState::kReady && state != ValidatorServoState::kServoOff) {
          AddDiagnostic(&result, program, command.line, DiagnosticCode::kInvalidRuntimeState,
                        "STOP requires ready or servo_off state");
          continue;
        }
        state = ValidatorServoState::kStopped;
        break;
      case CommandType::kNone:
        AddDiagnostic(&result, program, command.line, DiagnosticCode::kTaskSyntax,
                      "command type is missing");
        break;
    }
  }

  if (state != ValidatorServoState::kStopped) {
    AddDiagnostic(&result, program, 0, DiagnosticCode::kInvalidRuntimeState,
                  "program must end with STOP");
  }
  return result;
}

}  // namespace roborun
