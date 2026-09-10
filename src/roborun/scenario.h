#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "roborun/types.h"

namespace roborun {

enum class MotionPlanOutcome {
  kImmediateSuccess,
  kDelayedSuccess,
  kRejected,
  kFrozen,
};

enum class StopPlanOutcome {
  kImmediateConfirmation,
  kDelayedConfirmation,
  kRejected,
  kNeverConfirmed,
  kFailed,
};

struct StopPlan {
  std::size_t stop_episode = 0;
  StopPlanOutcome outcome = StopPlanOutcome::kImmediateConfirmation;
  BusinessTime delay_ms = 0;
};

struct MotionPlan {
  std::size_t movej_occurrence = 0;
  MotionPlanOutcome outcome = MotionPlanOutcome::kImmediateSuccess;
  BusinessTime delay_ms = 0;
};

enum class ScenarioEventKind {
  kDiChange,
  kToolFeedback,
  kConnectionLoss,
};

struct ScenarioEvent {
  BusinessTime time_ms = 0;
  ScenarioEventKind kind = ScenarioEventKind::kDiChange;
  std::string name;
  bool value = false;
  ToolPosition tool_position = ToolPosition::kOpen;
};

struct ScenarioControl {
  BusinessTime time_ms = 0;
  ControlType type = ControlType::kNone;
  std::string source;
};

struct MockScenario {
  std::string source;
  int schema_version = 1;
  std::map<std::size_t, MotionPlan> motion_plans;
  std::map<std::size_t, StopPlan> stop_plans;
  std::vector<ScenarioEvent> events;
  std::vector<ScenarioControl> controls;
};

struct ScenarioLoadResult {
  std::optional<MockScenario> scenario;
  std::vector<Diagnostic> diagnostics;
};

ScenarioLoadResult LoadMockScenario(const std::string& path);

}  // namespace roborun
