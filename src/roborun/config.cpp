#include "roborun/config.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <regex>
#include <string>
#include <unordered_set>
#include <utility>

namespace roborun {
namespace {

using Json = nlohmann::json;

struct JsonDocument {
  std::optional<Json> value;
  std::vector<Diagnostic> diagnostics;
};

void AddDiagnostic(std::vector<Diagnostic>* diagnostics, DiagnosticCode code,
                   const std::string& source, const std::string& path, const std::string& message,
                   std::size_t line = 0, std::size_t column = 0) {
  diagnostics->push_back(Diagnostic{code, source, line, column, path, message});
}

std::pair<std::size_t, std::size_t> LineColumnAt(const std::string& text, std::size_t byte) {
  std::size_t line = 1;
  std::size_t column = 1;
  const std::size_t end = std::min(byte, text.size());
  for (std::size_t index = 0; index < end; ++index) {
    if (text[index] == '\n') {
      ++line;
      column = 1;
    } else {
      ++column;
    }
  }
  return {line, column};
}

JsonDocument ReadJson(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    JsonDocument result;
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kConfigIo, path, "",
                  "cannot open configuration file");
    return result;
  }

  const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  try {
    return JsonDocument{Json::parse(text), {}};
  } catch (const Json::parse_error& error) {
    JsonDocument result;
    const auto [line, column] = LineColumnAt(text, error.byte == 0 ? 0 : error.byte - 1);
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kJsonSyntax, path, "", error.what(), line,
                  column);
    return result;
  } catch (const Json::exception& error) {
    JsonDocument result;
    const std::string message = error.what();
    std::size_t byte = 0;
    const std::size_t prefix = message.find("parsing '");
    if (prefix != std::string::npos) {
      const std::size_t token_begin = prefix + std::string("parsing '").size();
      const std::size_t token_end = message.find('\'', token_begin);
      if (token_end != std::string::npos) {
        const std::size_t token_position =
            text.find(message.substr(token_begin, token_end - token_begin));
        if (token_position != std::string::npos) {
          byte = token_position;
        }
      }
    }
    const auto [line, column] = LineColumnAt(text, byte);
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kJsonSyntax, path, "", message, line,
                  column);
    return result;
  }
}

const Json* RequiredMember(const Json& object, const std::string& name, const std::string& source,
                           const std::string& path, std::vector<Diagnostic>* diagnostics) {
  const std::string member_path = path + "/" + name;
  if (!object.is_object()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidConfiguration, source, path,
                  "expected an object");
    return nullptr;
  }
  const auto iterator = object.find(name);
  if (iterator == object.end()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidConfiguration, source, member_path,
                  "required field is missing");
    return nullptr;
  }
  return &(*iterator);
}

bool ReadSchemaVersion(const Json& root, const std::string& source,
                       std::vector<Diagnostic>* diagnostics) {
  const Json* version = RequiredMember(root, "schema_version", source, "", diagnostics);
  if (version == nullptr || (!version->is_number_integer() && !version->is_number_unsigned())) {
    if (version != nullptr) {
      AddDiagnostic(diagnostics, DiagnosticCode::kSchemaMismatch, source, "/schema_version",
                    "schema_version must be integer 1");
    }
    return false;
  }
  const bool supported = (version->is_number_integer() && version->get<int64_t>() == 1) ||
                         (version->is_number_unsigned() && version->get<uint64_t>() == 1);
  if (!supported) {
    AddDiagnostic(diagnostics, DiagnosticCode::kSchemaMismatch, source, "/schema_version",
                  "unsupported schema_version");
    return false;
  }
  return true;
}

bool ReadString(const Json* value, const std::string& source, const std::string& path,
                std::vector<Diagnostic>* diagnostics, std::string* output) {
  if (value == nullptr || !value->is_string()) {
    if (value != nullptr) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidConfiguration, source, path,
                    "expected a string");
    }
    return false;
  }
  *output = value->get<std::string>();
  if (output->empty()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidConfiguration, source, path,
                  "value must not be empty");
    return false;
  }
  return true;
}

bool ReadBoolean(const Json* value, const std::string& source, const std::string& path,
                 std::vector<Diagnostic>* diagnostics, bool* output) {
  if (value == nullptr || !value->is_boolean()) {
    if (value != nullptr) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidConfiguration, source, path,
                    "expected a boolean");
    }
    return false;
  }
  *output = value->get<bool>();
  return true;
}

bool ReadFiniteNumber(const Json* value, const std::string& source, const std::string& path,
                      std::vector<Diagnostic>* diagnostics, double* output,
                      DiagnosticCode nonfinite_code = DiagnosticCode::kInvalidConfiguration) {
  if (value == nullptr || !value->is_number()) {
    if (value != nullptr) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidConfiguration, source, path,
                    "expected a number");
    }
    return false;
  }
  *output = value->get<double>();
  if (!std::isfinite(*output)) {
    AddDiagnostic(diagnostics, nonfinite_code, source, path, "value must be finite");
    return false;
  }
  return true;
}

bool ReadJointValues(const Json& value, const std::string& source, const std::string& path,
                     std::vector<Diagnostic>* diagnostics, JointPositions* joints) {
  if (!value.is_array()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidConfiguration, source, path,
                  "expected an array");
    return false;
  }
  if (value.size() != kJointCount) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidJointCount, source, path,
                  "expected exactly 6 joint values");
    return false;
  }
  bool valid = true;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    valid = ReadFiniteNumber(&value[index], source, path + "/" + std::to_string(index), diagnostics,
                             &(*joints)[index], DiagnosticCode::kNonFiniteJoint) &&
            valid;
  }
  return valid;
}

}  // namespace

bool IsValidPointName(const std::string& name) {
  static const std::regex kPointNamePattern("^[A-Z][A-Z0-9_]{0,63}$");
  return std::regex_match(name, kPointNamePattern);
}

RobotConfigLoadResult LoadRobotConfiguration(const std::string& path) {
  RobotConfigLoadResult result;
  JsonDocument document = ReadJson(path);
  result.diagnostics = std::move(document.diagnostics);
  if (!document.value.has_value()) {
    return result;
  }

  const Json& root = *document.value;
  bool valid = root.is_object();
  if (!valid) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidConfiguration, path, "",
                  "configuration root must be an object");
  }
  valid = ReadSchemaVersion(root, path, &result.diagnostics) && valid;

  RobotConfiguration configuration;
  valid = ReadString(RequiredMember(root, "robot_id", path, "", &result.diagnostics), path,
                     "/robot_id", &result.diagnostics, &configuration.robot_id) &&
          valid;

  const Json* joints = RequiredMember(root, "joints", path, "", &result.diagnostics);
  if (joints == nullptr || !joints->is_array()) {
    if (joints != nullptr) {
      AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidConfiguration, path, "/joints",
                    "expected an array");
    }
    valid = false;
  } else if (joints->size() != kJointCount) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidJointCount, path, "/joints",
                  "expected exactly 6 joints");
    valid = false;
  } else {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const std::string joint_path = "/joints/" + std::to_string(index);
      const Json& joint = (*joints)[index];
      bool joint_valid = joint.is_object();
      if (!joint_valid) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidConfiguration, path, joint_path,
                      "joint must be an object");
      }
      joint_valid = ReadString(RequiredMember(joint, "name", path, joint_path, &result.diagnostics),
                               path, joint_path + "/name", &result.diagnostics,
                               &configuration.joints[index].name) &&
                    joint_valid;
      joint_valid =
          ReadFiniteNumber(RequiredMember(joint, "min", path, joint_path, &result.diagnostics),
                           path, joint_path + "/min", &result.diagnostics,
                           &configuration.joints[index].minimum) &&
          joint_valid;
      joint_valid =
          ReadFiniteNumber(RequiredMember(joint, "max", path, joint_path, &result.diagnostics),
                           path, joint_path + "/max", &result.diagnostics,
                           &configuration.joints[index].maximum) &&
          joint_valid;
      if (joint_valid &&
          configuration.joints[index].minimum >= configuration.joints[index].maximum) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kJointLimitViolation, path, joint_path,
                      "joint minimum must be less than maximum");
        joint_valid = false;
      }
      valid = joint_valid && valid;
    }
  }

  const bool reference_speed_valid = ReadFiniteNumber(
      RequiredMember(root, "reference_joint_speed", path, "", &result.diagnostics), path,
      "/reference_joint_speed", &result.diagnostics, &configuration.reference_joint_speed);
  valid = reference_speed_valid && valid;
  if (reference_speed_valid && configuration.reference_joint_speed <= 0.0) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidConfiguration, path,
                  "/reference_joint_speed", "reference_joint_speed must be positive");
    valid = false;
  }

  if (valid && result.diagnostics.empty()) {
    result.configuration = std::move(configuration);
  }
  return result;
}

PointCatalogLoadResult LoadPointCatalog(const std::string& path) {
  PointCatalogLoadResult result;
  JsonDocument document = ReadJson(path);
  result.diagnostics = std::move(document.diagnostics);
  if (!document.value.has_value()) {
    return result;
  }

  const Json& root = *document.value;
  bool valid = root.is_object();
  if (!valid) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidConfiguration, path, "",
                  "configuration root must be an object");
  }
  valid = ReadSchemaVersion(root, path, &result.diagnostics) && valid;

  PointCatalog catalog;
  catalog.source = path;
  valid = ReadString(RequiredMember(root, "robot_id", path, "", &result.diagnostics), path,
                     "/robot_id", &result.diagnostics, &catalog.robot_id) &&
          valid;
  const Json* points = RequiredMember(root, "points", path, "", &result.diagnostics);
  if (points == nullptr || !points->is_array()) {
    if (points != nullptr) {
      AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidConfiguration, path, "/points",
                    "expected an array");
    }
    valid = false;
  } else {
    std::unordered_set<std::string> names;
    for (std::size_t index = 0; index < points->size(); ++index) {
      const std::string point_path = "/points/" + std::to_string(index);
      const Json& point = (*points)[index];
      bool point_valid = point.is_object();
      if (!point_valid) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidConfiguration, path, point_path,
                      "point must be an object");
      }
      std::string name;
      point_valid = ReadString(RequiredMember(point, "name", path, point_path, &result.diagnostics),
                               path, point_path + "/name", &result.diagnostics, &name) &&
                    point_valid;
      if (point_valid && !IsValidPointName(name)) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidPointName, path,
                      point_path + "/name",
                      "point name must be an uppercase identifier of at most 64 characters");
        point_valid = false;
      }
      if (point_valid && !names.insert(name).second) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kDuplicatePoint, path,
                      point_path + "/name", "point name is duplicated: " + name);
        point_valid = false;
      }
      JointPositions joints{};
      const Json* joint_values =
          RequiredMember(point, "joints", path, point_path, &result.diagnostics);
      point_valid = joint_values != nullptr &&
                    ReadJointValues(*joint_values, path, point_path + "/joints",
                                    &result.diagnostics, &joints) &&
                    point_valid;
      if (point_valid) {
        catalog.points.emplace(std::move(name),
                               PointCatalog::Point{joints, point_path + "/joints"});
      }
      valid = point_valid && valid;
    }
  }

  if (valid && result.diagnostics.empty()) {
    result.catalog = std::move(catalog);
  }
  return result;
}

IOConfigLoadResult LoadIOConfiguration(const std::string& path) {
  IOConfigLoadResult result;
  JsonDocument document = ReadJson(path);
  result.diagnostics = std::move(document.diagnostics);
  if (!document.value.has_value()) {
    return result;
  }

  const Json& root = *document.value;
  bool valid = root.is_object();
  if (!valid) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidSignalConfiguration, path, "",
                  "configuration root must be an object");
  }
  valid = ReadSchemaVersion(root, path, &result.diagnostics) && valid;
  IOConfiguration configuration;
  configuration.source = path;
  const Json* signals = RequiredMember(root, "signals", path, "", &result.diagnostics);
  if (signals == nullptr || !signals->is_array()) {
    if (signals != nullptr) {
      AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidSignalConfiguration, path,
                    "/signals", "expected an array");
    }
    valid = false;
  } else {
    for (std::size_t index = 0; index < signals->size(); ++index) {
      const std::string item_path = "/signals/" + std::to_string(index);
      const Json& item = (*signals)[index];
      bool item_valid = item.is_object();
      if (!item_valid) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidSignalConfiguration, path,
                      item_path, "signal must be an object");
      }
      std::string name;
      item_valid = ReadString(RequiredMember(item, "name", path, item_path, &result.diagnostics),
                              path, item_path + "/name", &result.diagnostics, &name) &&
                   item_valid;
      if (item_valid && !IsValidPointName(name)) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidSignalConfiguration, path,
                      item_path + "/name", "signal name must be an uppercase identifier");
        item_valid = false;
      }
      std::string direction;
      item_valid =
          ReadString(RequiredMember(item, "direction", path, item_path, &result.diagnostics), path,
                     item_path + "/direction", &result.diagnostics, &direction) &&
          item_valid;
      SignalDirection signal_direction = SignalDirection::kDi;
      if (direction == "DI") {
        signal_direction = SignalDirection::kDi;
      } else if (direction == "DO") {
        signal_direction = SignalDirection::kDo;
      } else if (!direction.empty()) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidSignalConfiguration, path,
                      item_path + "/direction", "direction must be DI or DO");
        item_valid = false;
      } else {
        item_valid = false;
      }
      bool initial = false;
      item_valid =
          ReadBoolean(RequiredMember(item, "initial", path, item_path, &result.diagnostics), path,
                      item_path + "/initial", &result.diagnostics, &initial) &&
          item_valid;
      if (item_valid && configuration.signals.contains(name)) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kDuplicateSignal, path,
                      item_path + "/name", "signal name is duplicated: " + name);
        item_valid = false;
      }
      if (item_valid) {
        configuration.signals.emplace(name, DigitalSignal{name, signal_direction, initial, 0});
      }
      valid = item_valid && valid;
    }
  }
  if (valid && result.diagnostics.empty()) {
    result.configuration = std::move(configuration);
  }
  return result;
}

ToolConfigLoadResult LoadToolConfiguration(const std::string& path) {
  ToolConfigLoadResult result;
  JsonDocument document = ReadJson(path);
  result.diagnostics = std::move(document.diagnostics);
  if (!document.value.has_value()) {
    return result;
  }

  const Json& root = *document.value;
  bool valid = root.is_object();
  if (!valid) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidToolConfiguration, path, "",
                  "configuration root must be an object");
  }
  valid = ReadSchemaVersion(root, path, &result.diagnostics) && valid;
  ToolConfiguration configuration;
  configuration.source = path;
  const Json* tools = RequiredMember(root, "tools", path, "", &result.diagnostics);
  if (tools == nullptr || !tools->is_array()) {
    if (tools != nullptr) {
      AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidToolConfiguration, path, "/tools",
                    "expected an array");
    }
    valid = false;
  } else {
    for (std::size_t index = 0; index < tools->size(); ++index) {
      const std::string item_path = "/tools/" + std::to_string(index);
      const Json& item = (*tools)[index];
      bool item_valid = item.is_object();
      if (!item_valid) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidToolConfiguration, path,
                      item_path, "tool must be an object");
      }
      std::string name;
      item_valid = ReadString(RequiredMember(item, "name", path, item_path, &result.diagnostics),
                              path, item_path + "/name", &result.diagnostics, &name) &&
                   item_valid;
      if (item_valid && !IsValidPointName(name)) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidToolConfiguration, path,
                      item_path + "/name", "tool name must be an uppercase identifier");
        item_valid = false;
      }
      std::string type;
      item_valid = ReadString(RequiredMember(item, "type", path, item_path, &result.diagnostics),
                              path, item_path + "/type", &result.diagnostics, &type) &&
                   item_valid;
      if (item_valid && type != "gripper") {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidToolConfiguration, path,
                      item_path + "/type", "only gripper tools are supported");
        item_valid = false;
      }
      std::string commanded_text;
      item_valid =
          ReadString(RequiredMember(item, "commanded_state", path, item_path, &result.diagnostics),
                     path, item_path + "/commanded_state", &result.diagnostics, &commanded_text) &&
          item_valid;
      std::string observed_text;
      item_valid =
          ReadString(RequiredMember(item, "feedback_state", path, item_path, &result.diagnostics),
                     path, item_path + "/feedback_state", &result.diagnostics, &observed_text) &&
          item_valid;
      auto parse_position = [&](const std::string& text, const std::string& field,
                                ToolPosition* position) {
        if (text == "OPEN") {
          *position = ToolPosition::kOpen;
          return true;
        }
        if (text == "CLOSED") {
          *position = ToolPosition::kClosed;
          return true;
        }
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidToolState, path, field,
                      "tool state must be OPEN or CLOSED");
        return false;
      };
      ToolPosition commanded = ToolPosition::kOpen;
      ToolPosition observed = ToolPosition::kOpen;
      item_valid =
          parse_position(commanded_text, item_path + "/commanded_state", &commanded) && item_valid;
      item_valid =
          parse_position(observed_text, item_path + "/feedback_state", &observed) && item_valid;
      if (item_valid && configuration.tools.contains(name)) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidToolConfiguration, path,
                      item_path + "/name", "tool name is duplicated: " + name);
        item_valid = false;
      }
      if (item_valid) {
        configuration.tools.emplace(name,
                                    ToolState{name, ToolType::kGripper, commanded, observed, 0});
      }
      valid = item_valid && valid;
    }
  }
  if (valid && result.diagnostics.empty()) {
    result.configuration = std::move(configuration);
  }
  return result;
}

std::vector<Diagnostic> ValidatePointCatalog(const PointCatalog& catalog,
                                             const RobotConfiguration& robot) {
  std::vector<Diagnostic> diagnostics;
  if (catalog.robot_id != robot.robot_id) {
    AddDiagnostic(&diagnostics, DiagnosticCode::kRobotMismatch, catalog.source, "/robot_id",
                  "point catalog robot_id does not match robot configuration");
  }
  for (const auto& [name, point] : catalog.points) {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const JointLimit& limit = robot.joints[index];
      if (point.joints[index] < limit.minimum || point.joints[index] > limit.maximum) {
        AddDiagnostic(&diagnostics, DiagnosticCode::kJointLimitViolation, catalog.source,
                      point.joints_path + "/" + std::to_string(index),
                      "point joint " + std::to_string(index + 1) + " exceeds robot limit");
      }
    }
  }
  return diagnostics;
}

}  // namespace roborun
