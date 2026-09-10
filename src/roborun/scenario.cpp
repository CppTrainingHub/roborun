#include "roborun/scenario.h"

#include <fstream>
#include <initializer_list>
#include <limits>
#include <nlohmann/json.hpp>
#include <utility>

namespace roborun {
namespace {

using Json = nlohmann::json;

void Add(std::vector<Diagnostic>* diagnostics, DiagnosticCode code, const std::string& source,
         const std::string& path, const std::string& message) {
  diagnostics->push_back(Diagnostic{code, source, 0, 0, path, message});
}

void RejectUnknownFields(const Json& object, std::initializer_list<const char*> allowed,
                         const std::string& source, const std::string& path,
                         std::vector<Diagnostic>* diagnostics) {
  if (!object.is_object()) {
    return;
  }
  for (const auto& [key, value] : object.items()) {
    bool known = false;
    for (const char* field : allowed) {
      if (key == field) {
        known = true;
        break;
      }
    }
    if (!known) {
      Add(diagnostics, DiagnosticCode::kInvalidScenario, source, path + "/" + key, "unknown field");
    }
  }
}

const Json* Required(const Json& object, const char* key, const std::string& source,
                     const std::string& path, std::vector<Diagnostic>* diagnostics) {
  if (!object.is_object()) {
    Add(diagnostics, DiagnosticCode::kInvalidScenario, source, path, "expected an object");
    return nullptr;
  }
  const auto iterator = object.find(key);
  if (iterator == object.end()) {
    Add(diagnostics, DiagnosticCode::kInvalidScenario, source, path + "/" + key,
        "required field is missing");
    return nullptr;
  }
  return &*iterator;
}

bool ReadNonNegative(const Json* value, BusinessTime* result, const std::string& source,
                     const std::string& path, std::vector<Diagnostic>* diagnostics) {
  if (value == nullptr || (!value->is_number_integer() && !value->is_number_unsigned())) {
    if (value != nullptr) {
      Add(diagnostics, DiagnosticCode::kInvalidScenario, source, path,
          "expected a non-negative integer");
    }
    return false;
  }
  if (value->is_number_integer()) {
    const auto parsed = value->get<std::int64_t>();
    if (parsed < 0) {
      Add(diagnostics, DiagnosticCode::kInvalidScenario, source, path,
          "value must be non-negative");
      return false;
    }
    *result = parsed;
    return true;
  }
  const auto parsed = value->get<std::uint64_t>();
  if (parsed > static_cast<std::uint64_t>(std::numeric_limits<BusinessTime>::max())) {
    Add(diagnostics, DiagnosticCode::kInvalidScenario, source, path, "value is too large");
    return false;
  }
  *result = static_cast<BusinessTime>(parsed);
  return true;
}

bool ReadName(const Json* value, std::string* result, const std::string& source,
              const std::string& path, std::vector<Diagnostic>* diagnostics) {
  if (value == nullptr || !value->is_string() || value->get<std::string>().empty()) {
    if (value != nullptr) {
      Add(diagnostics, DiagnosticCode::kInvalidScenario, source, path,
          "expected a non-empty string");
    }
    return false;
  }
  *result = value->get<std::string>();
  return true;
}

}  // namespace

ScenarioLoadResult LoadMockScenario(const std::string& path) {
  ScenarioLoadResult result;
  std::ifstream input(path);
  if (!input) {
    Add(&result.diagnostics, DiagnosticCode::kConfigIo, path, "", "cannot open scenario file");
    return result;
  }
  Json root;
  try {
    input >> root;
  } catch (const Json::exception& error) {
    Add(&result.diagnostics, DiagnosticCode::kJsonSyntax, path, "", error.what());
    return result;
  }
  bool valid = root.is_object();
  if (!valid) {
    Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, "",
        "scenario root must be an object");
    return result;
  }
  const std::size_t root_diagnostics = result.diagnostics.size();
  RejectUnknownFields(root, {"schema_version", "motions", "events", "controls", "stops"}, path, "",
                      &result.diagnostics);
  valid = valid && result.diagnostics.size() == root_diagnostics;
  const Json* version = Required(root, "schema_version", path, "", &result.diagnostics);
  if (version == nullptr || (!version->is_number_integer() && !version->is_number_unsigned()) ||
      (version->is_number_integer() &&
       (version->get<std::int64_t>() < 1 || version->get<std::int64_t>() > 2)) ||
      (version->is_number_unsigned() &&
       (version->get<std::uint64_t>() < 1 || version->get<std::uint64_t>() > 2))) {
    Add(&result.diagnostics, DiagnosticCode::kSchemaMismatch, path, "/schema_version",
        "scenario schema_version must be 1 or 2");
    valid = false;
  }
  MockScenario scenario;
  scenario.source = path;
  if (version != nullptr && version->is_number_integer()) {
    scenario.schema_version = static_cast<int>(version->get<std::int64_t>());
  } else if (version != nullptr && version->is_number_unsigned()) {
    scenario.schema_version = static_cast<int>(version->get<std::uint64_t>());
  }
  if (scenario.schema_version == 1 && (root.contains("controls") || root.contains("stops"))) {
    Add(&result.diagnostics, DiagnosticCode::kSchemaMismatch, path, "/schema_version",
        "scenario controls and stops require schema_version 2");
    valid = false;
  }

  const Json* motions = root.contains("motions") ? &root["motions"] : nullptr;
  if (motions != nullptr && !motions->is_array()) {
    Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, "/motions",
        "motions must be an array");
    valid = false;
  }
  if (motions != nullptr && motions->is_array()) {
    for (std::size_t index = 0; index < motions->size(); ++index) {
      const Json& item = (*motions)[index];
      const std::string item_path = "/motions/" + std::to_string(index);
      const std::size_t item_diagnostics = result.diagnostics.size();
      RejectUnknownFields(item, {"movej", "outcome", "delay_ms"}, path, item_path,
                          &result.diagnostics);
      valid = valid && result.diagnostics.size() == item_diagnostics;
      BusinessTime occurrence_time = 0;
      const Json* occurrence = Required(item, "movej", path, item_path, &result.diagnostics);
      if (occurrence == nullptr || !occurrence->is_number_integer() ||
          occurrence->get<std::int64_t>() <= 0) {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/movej",
            "movej occurrence must be a positive integer");
        valid = false;
      } else {
        occurrence_time = occurrence->get<std::int64_t>();
      }
      std::string outcome;
      const Json* outcome_value = Required(item, "outcome", path, item_path, &result.diagnostics);
      if (outcome_value == nullptr || !outcome_value->is_string()) {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/outcome",
            "outcome is required");
        valid = false;
      } else {
        outcome = outcome_value->get<std::string>();
      }
      MotionPlan plan;
      plan.movej_occurrence = static_cast<std::size_t>(occurrence_time);
      if (outcome == "immediate_success") {
        plan.outcome = MotionPlanOutcome::kImmediateSuccess;
      } else if (outcome == "delayed_success") {
        plan.outcome = MotionPlanOutcome::kDelayedSuccess;
        if (!ReadNonNegative(Required(item, "delay_ms", path, item_path, &result.diagnostics),
                             &plan.delay_ms, path, item_path + "/delay_ms", &result.diagnostics)) {
          valid = false;
        }
      } else if (outcome == "rejected") {
        plan.outcome = MotionPlanOutcome::kRejected;
      } else if (outcome == "frozen") {
        plan.outcome = MotionPlanOutcome::kFrozen;
      } else {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/outcome",
            "unknown motion outcome");
        valid = false;
      }
      if (scenario.motion_plans.contains(plan.movej_occurrence)) {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/movej",
            "motion occurrence has more than one plan");
        valid = false;
      } else if (occurrence_time > 0) {
        scenario.motion_plans.emplace(plan.movej_occurrence, plan);
      }
    }
  }

  const Json* events = root.contains("events") ? &root["events"] : nullptr;
  if (events != nullptr && !events->is_array()) {
    Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, "/events",
        "events must be an array");
    valid = false;
  }
  BusinessTime previous_time = 0;
  if (events != nullptr && events->is_array()) {
    for (std::size_t index = 0; index < events->size(); ++index) {
      const Json& item = (*events)[index];
      const std::string item_path = "/events/" + std::to_string(index);
      const std::size_t item_diagnostics = result.diagnostics.size();
      BusinessTime time = 0;
      if (!ReadNonNegative(Required(item, "time_ms", path, item_path, &result.diagnostics), &time,
                           path, item_path + "/time_ms", &result.diagnostics)) {
        valid = false;
      }
      if (time < previous_time) {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/time_ms",
            "event times must not move backward");
        valid = false;
      }
      previous_time = time;
      std::string kind;
      const Json* kind_value = Required(item, "kind", path, item_path, &result.diagnostics);
      if (kind_value == nullptr || !kind_value->is_string()) {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/kind",
            "event kind is required");
        valid = false;
        continue;
      }
      kind = kind_value->get<std::string>();
      ScenarioEvent event;
      event.time_ms = time;
      if (kind == "DI") {
        RejectUnknownFields(item, {"time_ms", "kind", "name", "value"}, path, item_path,
                            &result.diagnostics);
        event.kind = ScenarioEventKind::kDiChange;
        if (!ReadName(Required(item, "name", path, item_path, &result.diagnostics), &event.name,
                      path, item_path + "/name", &result.diagnostics) ||
            !item.contains("value") || !item["value"].is_boolean()) {
          Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path,
              "DI event requires name and boolean value");
          valid = false;
        } else {
          event.value = item["value"].get<bool>();
        }
      } else if (kind == "TOOL") {
        RejectUnknownFields(item, {"time_ms", "kind", "name", "state"}, path, item_path,
                            &result.diagnostics);
        event.kind = ScenarioEventKind::kToolFeedback;
        std::string position;
        if (!ReadName(Required(item, "name", path, item_path, &result.diagnostics), &event.name,
                      path, item_path + "/name", &result.diagnostics) ||
            !ReadName(Required(item, "state", path, item_path, &result.diagnostics), &position,
                      path, item_path + "/state", &result.diagnostics) ||
            (position != "OPEN" && position != "CLOSED")) {
          Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path,
              "TOOL event requires name and OPEN/CLOSED state");
          valid = false;
        } else {
          event.tool_position = position == "OPEN" ? ToolPosition::kOpen : ToolPosition::kClosed;
        }
      } else if (kind == "CONNECTION_LOSS") {
        RejectUnknownFields(item, {"time_ms", "kind"}, path, item_path, &result.diagnostics);
        event.kind = ScenarioEventKind::kConnectionLoss;
      } else {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/kind",
            "unknown event kind");
        valid = false;
      }
      valid = valid && result.diagnostics.size() == item_diagnostics;
      scenario.events.push_back(std::move(event));
    }
  }

  const Json* stops = root.contains("stops") ? &root["stops"] : nullptr;
  if (stops != nullptr && !stops->is_array()) {
    Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, "/stops",
        "stops must be an array");
    valid = false;
  }
  if (stops != nullptr && stops->is_array()) {
    for (std::size_t index = 0; index < stops->size(); ++index) {
      const Json& item = (*stops)[index];
      const std::string item_path = "/stops/" + std::to_string(index);
      RejectUnknownFields(item, {"episode", "outcome", "delay_ms"}, path, item_path,
                          &result.diagnostics);
      const Json* episode = Required(item, "episode", path, item_path, &result.diagnostics);
      if (episode == nullptr || !episode->is_number_integer() ||
          episode->get<std::int64_t>() <= 0) {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/episode",
            "stop episode must be a positive integer");
        valid = false;
        continue;
      }
      StopPlan plan;
      plan.stop_episode = static_cast<std::size_t>(episode->get<std::int64_t>());
      const Json* outcome_value = Required(item, "outcome", path, item_path, &result.diagnostics);
      const std::string outcome = outcome_value != nullptr && outcome_value->is_string()
                                      ? outcome_value->get<std::string>()
                                      : "";
      if (outcome == "immediate_confirmation") {
        plan.outcome = StopPlanOutcome::kImmediateConfirmation;
      } else if (outcome == "delayed_confirmation") {
        plan.outcome = StopPlanOutcome::kDelayedConfirmation;
        if (!ReadNonNegative(Required(item, "delay_ms", path, item_path, &result.diagnostics),
                             &plan.delay_ms, path, item_path + "/delay_ms", &result.diagnostics)) {
          valid = false;
        }
      } else if (outcome == "rejected") {
        plan.outcome = StopPlanOutcome::kRejected;
      } else if (outcome == "never_confirmed") {
        plan.outcome = StopPlanOutcome::kNeverConfirmed;
      } else if (outcome == "failed") {
        plan.outcome = StopPlanOutcome::kFailed;
      } else {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/outcome",
            "unknown stop outcome");
        valid = false;
      }
      if (scenario.stop_plans.contains(plan.stop_episode)) {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/episode",
            "stop episode has more than one plan");
        valid = false;
      } else {
        scenario.stop_plans.emplace(plan.stop_episode, plan);
      }
    }
  }

  const Json* controls = root.contains("controls") ? &root["controls"] : nullptr;
  if (controls != nullptr && !controls->is_array()) {
    Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, "/controls",
        "controls must be an array");
    valid = false;
  }
  BusinessTime previous_control_time = 0;
  if (controls != nullptr && controls->is_array()) {
    for (std::size_t index = 0; index < controls->size(); ++index) {
      const Json& item = (*controls)[index];
      const std::string item_path = "/controls/" + std::to_string(index);
      RejectUnknownFields(item, {"time_ms", "action", "source"}, path, item_path,
                          &result.diagnostics);
      ScenarioControl control;
      if (!ReadNonNegative(Required(item, "time_ms", path, item_path, &result.diagnostics),
                           &control.time_ms, path, item_path + "/time_ms", &result.diagnostics)) {
        valid = false;
      }
      if (control.time_ms < previous_control_time) {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/time_ms",
            "control times must not move backward");
        valid = false;
      }
      previous_control_time = control.time_ms;
      const Json* action = Required(item, "action", path, item_path, &result.diagnostics);
      const std::string action_name =
          action != nullptr && action->is_string() ? action->get<std::string>() : "";
      if (action_name == "PAUSE") {
        control.type = ControlType::kPause;
      } else if (action_name == "RESUME") {
        control.type = ControlType::kResume;
      } else if (action_name == "STOP") {
        control.type = ControlType::kStop;
      } else if (action_name == "ESTOP") {
        control.type = ControlType::kEstop;
      } else if (action_name == "RESET") {
        control.type = ControlType::kReset;
      } else {
        Add(&result.diagnostics, DiagnosticCode::kInvalidScenario, path, item_path + "/action",
            "unknown control action");
        valid = false;
      }
      if (item.contains("source")) {
        if (!ReadName(&item["source"], &control.source, path, item_path + "/source",
                      &result.diagnostics)) {
          valid = false;
        }
      } else {
        control.source = "scenario";
      }
      scenario.controls.push_back(std::move(control));
    }
  }

  if (valid && result.diagnostics.empty()) {
    result.scenario = std::move(scenario);
  }
  return result;
}

}  // namespace roborun
