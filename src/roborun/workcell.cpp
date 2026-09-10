#include "roborun/workcell.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <utility>

namespace roborun {
namespace {

using Json = nlohmann::json;

void AddDiagnostic(std::vector<Diagnostic>* diagnostics, DiagnosticCode code,
                   const std::string& source, const std::string& path, const std::string& message) {
  diagnostics->push_back(Diagnostic{code, source, 0, 0, path, message});
}

bool ReadAlias(const Json& object, const char* name, const std::string& source,
               std::vector<Diagnostic>* diagnostics, std::string* value) {
  const auto iterator = object.find(name);
  if (iterator == object.end() || !iterator->is_string() || iterator->get<std::string>().empty() ||
      iterator->get<std::string>()[0] != '/') {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                  std::string("/") + name, "must be a non-empty absolute alias path");
    return false;
  }
  *value = iterator->get<std::string>();
  return true;
}

bool ReadFinitePositive(const Json& object, const char* name, const std::string& source,
                        std::vector<Diagnostic>* diagnostics, double* value) {
  const auto iterator = object.find(name);
  if (iterator == object.end() || !iterator->is_number()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                  std::string("/") + name, "must be a positive finite number");
    return false;
  }
  *value = iterator->get<double>();
  if (!std::isfinite(*value) || *value <= 0.0) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                  std::string("/") + name, "must be a positive finite number");
    return false;
  }
  return true;
}

bool ReadPositiveInteger(const Json& object, const char* name, const std::string& source,
                         std::vector<Diagnostic>* diagnostics, int* value) {
  const auto iterator = object.find(name);
  if (iterator == object.end() ||
      (!iterator->is_number_integer() && !iterator->is_number_unsigned())) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                  std::string("/") + name, "must be a positive integer");
    return false;
  }
  if (iterator->is_number_unsigned()) {
    const std::uint64_t parsed = iterator->get<std::uint64_t>();
    if (parsed == 0 || parsed > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                    std::string("/") + name, "must be a positive integer in range");
      return false;
    }
    *value = static_cast<int>(parsed);
    return true;
  }
  const std::int64_t parsed = iterator->get<std::int64_t>();
  if (parsed <= 0 || parsed > std::numeric_limits<int>::max()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                  std::string("/") + name, "must be a positive integer in range");
    return false;
  }
  *value = static_cast<int>(parsed);
  return true;
}

bool ReadPose(const Json& object, const char* name, const std::string& source,
              std::vector<Diagnostic>* diagnostics, Pose* pose) {
  const auto iterator = object.find(name);
  if (iterator == object.end() || !iterator->is_array() || iterator->size() != pose->size()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                  std::string("/") + name, "must be a seven-value [x,y,z,qx,qy,qz,qw] pose");
    return false;
  }
  for (std::size_t index = 0; index < pose->size(); ++index) {
    if (!(*iterator)[index].is_number()) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                    std::string("/") + name, "pose values must be finite numbers");
      return false;
    }
    (*pose)[index] = (*iterator)[index].get<double>();
    if (!std::isfinite((*pose)[index])) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                    std::string("/") + name, "pose values must be finite numbers");
      return false;
    }
  }
  const double quaternion_norm = std::sqrt((*pose)[3] * (*pose)[3] + (*pose)[4] * (*pose)[4] +
                                           (*pose)[5] * (*pose)[5] + (*pose)[6] * (*pose)[6]);
  if (quaternion_norm < 0.999 || quaternion_norm > 1.001) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                  std::string("/") + name, "pose quaternion must be normalized");
    return false;
  }
  return true;
}

bool IsAlias(const Json& value) {
  return value.is_string() && !value.get<std::string>().empty() &&
         value.get<std::string>().front() == '/';
}

bool IsSceneProperty(const Json& value) {
  return value.is_string() && value.get<std::string>().starts_with("signal.roborun.");
}

bool IsTolerance(double value) { return std::isfinite(value) && value > 0.0 && value <= 10.0; }

bool ReadSignalMappings(const Json& root, const std::string& source,
                        WorkcellConfiguration* configuration,
                        std::vector<Diagnostic>* diagnostics) {
  const auto iterator = root.find("signals");
  if (iterator == root.end() || !iterator->is_array()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/signals",
                  "must be an array of logical signal mappings");
    return false;
  }
  configuration->signals.clear();
  bool valid = true;
  for (std::size_t index = 0; index < iterator->size(); ++index) {
    const Json& entry = (*iterator)[index];
    const std::string entry_path = "/signals/" + std::to_string(index);
    if (!entry.is_object() || !entry.contains("name") || !entry["name"].is_string() ||
        entry["name"].get<std::string>().empty() || !entry.contains("property") ||
        !IsSceneProperty(entry["property"]) || !entry.contains("direction") ||
        !entry["direction"].is_string()) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, entry_path,
                    "requires name, di/do direction and signal.roborun.* property");
      valid = false;
      continue;
    }
    SignalDirection direction;
    const std::string direction_name = entry["direction"].get<std::string>();
    if (direction_name == "di") {
      direction = SignalDirection::kDi;
    } else if (direction_name == "do") {
      direction = SignalDirection::kDo;
    } else {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                    entry_path + "/direction", "must be di or do");
      valid = false;
      continue;
    }
    const std::string name = entry["name"].get<std::string>();
    if (configuration->signals.contains(name)) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, entry_path,
                    "logical signal name is duplicated: " + name);
      valid = false;
      continue;
    }
    configuration->signals.emplace(
        name, WorkcellSignalConfiguration{
                  DigitalSignal{name, direction, entry.value("initial", false), 0},
                  entry["property"].get<std::string>()});
  }
  return valid;
}

bool ReadToolMappings(const Json& root, const std::string& source,
                      WorkcellConfiguration* configuration, std::vector<Diagnostic>* diagnostics) {
  const auto iterator = root.find("tools");
  if (iterator == root.end() || !iterator->is_array()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/tools",
                  "must be an array of tool mappings");
    return false;
  }
  configuration->tools.clear();
  bool valid = true;
  for (std::size_t index = 0; index < iterator->size(); ++index) {
    const Json& entry = (*iterator)[index];
    const std::string entry_path = "/tools/" + std::to_string(index);
    if (!entry.is_object() || !entry.contains("name") || !entry["name"].is_string() ||
        entry["name"].get<std::string>().empty()) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, entry_path,
                    "requires a non-empty logical tool name");
      valid = false;
      continue;
    }
    WorkcellToolConfiguration tool;
    tool.name = entry["name"].get<std::string>();
    const std::array<std::pair<const char*, std::string*>, 3> aliases = {
        {{"alias", &tool.alias},
         {"joint_alias", &tool.joint_alias},
         {"tcp_alias", &tool.tcp_alias}}};
    for (const auto& [field, destination] : aliases) {
      if (!entry.contains(field) || !IsAlias(entry[field])) {
        AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                      entry_path + "/" + field, "must be a non-empty absolute alias path");
        valid = false;
      } else {
        *destination = entry[field].get<std::string>();
      }
    }
    const std::array<std::pair<const char*, double*>, 5> values = {
        {{"open_aperture", &tool.open_aperture},
         {"closed_aperture", &tool.closed_aperture},
         {"aperture_tolerance", &tool.aperture_tolerance},
         {"open_joint_position", &tool.open_joint_position},
         {"closed_joint_position", &tool.closed_joint_position}}};
    for (const auto& [field, destination] : values) {
      if (!entry.contains(field) || !entry[field].is_number() ||
          !std::isfinite(entry[field].get<double>())) {
        AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                      entry_path + "/" + field, "must be finite");
        valid = false;
      } else {
        *destination = entry[field].get<double>();
      }
    }
    if (tool.closed_aperture < 0.0 || tool.aperture_tolerance <= 0.0 ||
        tool.closed_aperture >= tool.open_aperture ||
        tool.closed_joint_position >= tool.open_joint_position) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, entry_path,
                    "tool opening and motor bounds are inconsistent");
      valid = false;
    }
    if (!entry.contains("command_property") || !entry["command_property"].is_string() ||
        !entry["command_property"].get<std::string>().starts_with("signal.")) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source,
                    entry_path + "/command_property", "must use a signal.* property");
      valid = false;
    } else {
      tool.command_property = entry["command_property"].get<std::string>();
    }
    if (configuration->tools.contains(tool.name)) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, entry_path,
                    "logical tool name is duplicated: " + tool.name);
      valid = false;
    } else {
      configuration->tools.emplace(tool.name, std::move(tool));
    }
  }
  if (configuration->tools.empty()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/tools",
                  "at least one logical tool mapping is required");
    valid = false;
  }
  return valid;
}

bool ReadCollisionRules(const Json& root, const std::string& source,
                        WorkcellConfiguration* configuration,
                        std::vector<Diagnostic>* diagnostics) {
  const auto iterator = root.find("collisions");
  if (iterator == root.end() || !iterator->is_array()) {
    AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/collisions",
                  "must be an array of collision rules");
    return false;
  }
  configuration->collisions.clear();
  bool valid = true;
  std::set<std::pair<std::string, std::string>> pairs;
  for (std::size_t index = 0; index < iterator->size(); ++index) {
    const Json& entry = (*iterator)[index];
    const std::string entry_path = "/collisions/" + std::to_string(index);
    if (!entry.is_object() || !entry.contains("first_alias") || !IsAlias(entry["first_alias"]) ||
        !entry.contains("second_alias") || !IsAlias(entry["second_alias"]) ||
        !entry.contains("permitted") || !entry["permitted"].is_boolean()) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, entry_path,
                    "requires two absolute aliases and a permitted Boolean");
      valid = false;
      continue;
    }
    std::string first = entry["first_alias"].get<std::string>();
    std::string second = entry["second_alias"].get<std::string>();
    if (first == second ||
        !pairs.emplace(std::min(first, second), std::max(first, second)).second) {
      AddDiagnostic(diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, entry_path,
                    "collision pair is self-referential or duplicated");
      valid = false;
      continue;
    }
    configuration->collisions.push_back(
        WorkcellCollisionRule{std::move(first), std::move(second), entry["permitted"].get<bool>()});
  }
  return valid;
}

}  // namespace

WorkcellConfiguration MakeDefaultWorkcellConfiguration() {
  WorkcellConfiguration configuration;
  configuration.source = "built-in:workcell";
  configuration.scene_schema = "roborun.workcell.v1";
  configuration.root_alias = "/RoboRun";
  configuration.robot_alias = "/RoboRun/UR5";
  configuration.tool_mount_alias = "/RoboRun/UR5/ToolMount";
  configuration.tool_alias = "/RoboRun/UR5/ToolMount/RG2";
  configuration.workpiece_alias = "/RoboRun/Workpiece";
  configuration.pick_alias = "/RoboRun/PICK";
  configuration.place_alias = "/RoboRun/PLACE";
  configuration.workpiece_initial_pose = {-0.2951062214, -0.3900437137, 0.9107258568, 0.3774108144,
                                          -0.6413791194, -0.0705928859, 0.6642368153};
  configuration.pick_pose = configuration.workpiece_initial_pose;
  configuration.place_pose = {-0.4047793199, -0.1161940509, 0.9722340129, -0.0015342691,
                              -0.7157843511, -0.0703726048, 0.6947647841};
  configuration.signals.emplace(
      "PART_READY",
      WorkcellSignalConfiguration{DigitalSignal{"PART_READY", SignalDirection::kDi, false, 0},
                                  "signal.roborun.part_ready"});
  configuration.signals.emplace(
      "CLAMP_ENABLE",
      WorkcellSignalConfiguration{DigitalSignal{"CLAMP_ENABLE", SignalDirection::kDo, false, 0},
                                  "signal.roborun.clamp_enable"});
  configuration.signals.emplace(
      "GRIP_OK",
      WorkcellSignalConfiguration{DigitalSignal{"GRIP_OK", SignalDirection::kDi, false, 0},
                                  "signal.roborun.grip_ok"});
  configuration.tools.emplace(
      "GRIPPER", WorkcellToolConfiguration{"GRIPPER", configuration.tool_alias,
                                           configuration.tool_alias + "/Joint",
                                           configuration.tool_alias + "/TCP", 0.085, 0.001, 0.002,
                                           0.05, -0.045, "signal.RG2_open"});
  return configuration;
}

WorkcellConfigLoadResult LoadWorkcellConfiguration(const std::string& path) {
  WorkcellConfigLoadResult result;
  std::ifstream input(path);
  if (!input) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kConfigIo, path, "",
                  "cannot open configuration");
    return result;
  }
  Json root;
  try {
    input >> root;
  } catch (const Json::parse_error& error) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kJsonSyntax, path, "", error.what());
    return result;
  }
  const auto schema_version = root.is_object() ? root.find("schema_version") : root.end();
  const bool schema_is_one =
      root.is_object() && schema_version != root.end() &&
      ((schema_version->is_number_unsigned() && schema_version->get<std::uint64_t>() == 1) ||
       (!schema_version->is_number_unsigned() && schema_version->is_number_integer() &&
        schema_version->get<std::int64_t>() == 1));
  if (!schema_is_one) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kWorkcellSchemaMismatch, path,
                  "/schema_version", "schema_version must be integer 1");
    return result;
  }

  WorkcellConfiguration configuration = MakeDefaultWorkcellConfiguration();
  configuration.source = path;
  const auto manifest_path = root.find("manifest_path");
  if (manifest_path != root.end()) {
    if (!manifest_path->is_string() || manifest_path->get<std::string>().empty()) {
      AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, path,
                    "/manifest_path", "must be a non-empty path");
      return result;
    }
    configuration.manifest_path =
        (std::filesystem::path(path).parent_path() / manifest_path->get<std::string>()).string();
  }
  bool valid = true;
  const auto scene_schema = root.find("scene_schema");
  if (scene_schema == root.end() || !scene_schema->is_string() ||
      scene_schema->get<std::string>().empty()) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, path,
                  "/scene_schema", "must be a non-empty string");
    valid = false;
  } else {
    configuration.scene_schema = scene_schema->get<std::string>();
  }
  valid =
      ReadAlias(root, "root_alias", path, &result.diagnostics, &configuration.root_alias) && valid;
  valid = ReadAlias(root, "robot_alias", path, &result.diagnostics, &configuration.robot_alias) &&
          valid;
  valid = ReadAlias(root, "tool_mount_alias", path, &result.diagnostics,
                    &configuration.tool_mount_alias) &&
          valid;
  valid =
      ReadAlias(root, "tool_alias", path, &result.diagnostics, &configuration.tool_alias) && valid;
  valid = ReadAlias(root, "workpiece_alias", path, &result.diagnostics,
                    &configuration.workpiece_alias) &&
          valid;
  valid =
      ReadAlias(root, "pick_alias", path, &result.diagnostics, &configuration.pick_alias) && valid;
  valid = ReadAlias(root, "place_alias", path, &result.diagnostics, &configuration.place_alias) &&
          valid;
  valid = ReadPose(root, "workpiece_initial_pose", path, &result.diagnostics,
                   &configuration.workpiece_initial_pose) &&
          valid;
  valid = ReadPose(root, "pick_pose", path, &result.diagnostics, &configuration.pick_pose) && valid;
  valid =
      ReadPose(root, "place_pose", path, &result.diagnostics, &configuration.place_pose) && valid;
  valid = ReadFinitePositive(root, "pick_radius", path, &result.diagnostics,
                             &configuration.pick_radius) &&
          valid;
  valid = ReadFinitePositive(root, "place_translation_tolerance", path, &result.diagnostics,
                             &configuration.place_translation_tolerance) &&
          valid;
  valid = ReadFinitePositive(root, "place_rotation_tolerance", path, &result.diagnostics,
                             &configuration.place_rotation_tolerance) &&
          valid;
  valid = ReadFinitePositive(root, "joint_tolerance_radians", path, &result.diagnostics,
                             &configuration.joint_tolerance_radians) &&
          valid;
  valid = ReadPositiveInteger(root, "stable_steps", path, &result.diagnostics,
                              &configuration.stable_steps) &&
          valid;
  const auto deadlines = root.find("default_deadlines_ms");
  if (deadlines == root.end() || !deadlines->is_object()) {
    AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, path,
                  "/default_deadlines_ms", "must be an object of positive deadlines");
    valid = false;
  } else {
    int move = 0;
    int tool = 0;
    int grip = 0;
    int safe_state = 0;
    valid = ReadPositiveInteger(*deadlines, "move", path, &result.diagnostics, &move) && valid;
    valid = ReadPositiveInteger(*deadlines, "tool", path, &result.diagnostics, &tool) && valid;
    valid = ReadPositiveInteger(*deadlines, "grip", path, &result.diagnostics, &grip) && valid;
    valid = ReadPositiveInteger(*deadlines, "safe_state", path, &result.diagnostics, &safe_state) &&
            valid;
    configuration.default_move_timeout_ms = move;
    configuration.default_tool_timeout_ms = tool;
    configuration.default_grip_timeout_ms = grip;
    configuration.default_safe_state_timeout_ms = safe_state;
  }
  valid = ReadSignalMappings(root, path, &configuration, &result.diagnostics) && valid;
  valid = ReadToolMappings(root, path, &configuration, &result.diagnostics) && valid;
  valid = ReadCollisionRules(root, path, &configuration, &result.diagnostics) && valid;
  if (!valid) {
    return result;
  }
  const std::vector<Diagnostic> manifest_diagnostics =
      ValidateWorkcellManifest(MakeWorkcellManifest(configuration), path);
  result.diagnostics.insert(result.diagnostics.end(), manifest_diagnostics.begin(),
                            manifest_diagnostics.end());
  if (result.diagnostics.empty() && !configuration.manifest_path.empty()) {
    std::ifstream manifest_input(configuration.manifest_path);
    if (!manifest_input) {
      AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, path,
                    "/manifest_path", "cannot open workcell manifest");
    } else {
      try {
        const Json published_manifest = Json::parse(manifest_input);
        const Json generated_manifest =
            Json::parse(SerializeWorkcellManifest(MakeWorkcellManifest(configuration)));
        if (published_manifest != generated_manifest) {
          AddDiagnostic(&result.diagnostics, DiagnosticCode::kWorkcellSchemaMismatch, path,
                        "/manifest_path", "published workcell manifest does not match config");
        }
      } catch (const Json::exception& error) {
        AddDiagnostic(&result.diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, path,
                      "/manifest_path", error.what());
      }
    }
  }
  if (result.diagnostics.empty()) {
    result.configuration = std::move(configuration);
  }
  return result;
}

WorkcellManifest MakeWorkcellManifest(const WorkcellConfiguration& configuration) {
  WorkcellManifest manifest;
  manifest.scene_schema = configuration.scene_schema;
  manifest.signals = configuration.signals;
  manifest.tools = configuration.tools;
  manifest.objects = {
      {configuration.root_alias, "dummy", "", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
      {configuration.robot_alias,
       "model",
       configuration.root_alias,
       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
      {configuration.tool_mount_alias,
       "dummy",
       configuration.robot_alias,
       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
      {configuration.tool_alias,
       "model",
       configuration.tool_mount_alias,
       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
  };
  if (!configuration.tools.empty()) {
    const WorkcellToolConfiguration& tool = configuration.tools.begin()->second;
    manifest.objects.push_back(
        {tool.joint_alias, "joint", configuration.tool_alias, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}});
    manifest.objects.push_back(
        {tool.tcp_alias, "dummy", configuration.tool_alias, {0.0, 0.0, 0.19, 0.0, 0.0, 0.0, 1.0}});
  }
  manifest.objects.push_back({configuration.workpiece_alias, "shape", configuration.root_alias,
                              configuration.workpiece_initial_pose});
  manifest.objects.push_back(
      {configuration.pick_alias, "dummy", configuration.root_alias, configuration.pick_pose});
  manifest.objects.push_back(
      {configuration.place_alias, "dummy", configuration.root_alias, configuration.place_pose});
  return manifest;
}

std::vector<Diagnostic> ValidateWorkcellManifest(const WorkcellManifest& manifest,
                                                 const std::string& source) {
  std::vector<Diagnostic> diagnostics;
  if (manifest.schema_version != 1) {
    AddDiagnostic(&diagnostics, DiagnosticCode::kWorkcellSchemaMismatch, source, "/schema_version",
                  "unsupported workcell manifest schema version");
  }
  if (manifest.scene_schema != "roborun.workcell.v1") {
    AddDiagnostic(&diagnostics, DiagnosticCode::kWorkcellSchemaMismatch, source, "/scene_schema",
                  "unsupported workcell scene schema");
  }
  std::set<std::string> aliases;
  for (const WorkcellObject& object : manifest.objects) {
    if (object.alias.empty() || object.alias[0] != '/') {
      AddDiagnostic(&diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/objects",
                    "object alias must be absolute");
      continue;
    }
    if (!aliases.insert(object.alias).second) {
      AddDiagnostic(&diagnostics, DiagnosticCode::kDuplicateWorkcellAlias, source, "/objects",
                    "object alias is duplicated: " + object.alias);
    }
  }
  if (manifest.signals.empty()) {
    AddDiagnostic(&diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/signals",
                  "at least one logical signal mapping is required");
  }
  for (const auto& [name, mapping] : manifest.signals) {
    if (name.empty() || mapping.signal.name != name ||
        !mapping.property.starts_with("signal.roborun.")) {
      AddDiagnostic(&diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/signals",
                    "signal mapping has an invalid logical name or property");
    }
  }
  if (manifest.tools.empty()) {
    AddDiagnostic(&diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/tools",
                  "at least one logical tool mapping is required");
  }
  for (const auto& [name, tool] : manifest.tools) {
    if (name.empty() || tool.name != name || tool.alias.empty() || tool.joint_alias.empty() ||
        tool.tcp_alias.empty() || tool.closed_aperture >= tool.open_aperture ||
        !IsTolerance(tool.aperture_tolerance) ||
        tool.closed_joint_position >= tool.open_joint_position ||
        !tool.command_property.starts_with("signal.")) {
      AddDiagnostic(&diagnostics, DiagnosticCode::kInvalidWorkcellConfiguration, source, "/tools",
                    "tool mapping has invalid aliases or aperture bounds");
    }
  }
  return diagnostics;
}

std::string SerializeWorkcellManifest(const WorkcellManifest& manifest) {
  Json root;
  root["schema_version"] = manifest.schema_version;
  root["scene_schema"] = manifest.scene_schema;
  root["objects"] = Json::array();
  for (const WorkcellObject& object : manifest.objects) {
    root["objects"].push_back({{"alias", object.alias},
                               {"type", object.type},
                               {"parent_alias", object.parent_alias},
                               {"pose", object.pose}});
  }
  root["signals"] = Json::array();
  for (const auto& [name, mapping] : manifest.signals) {
    root["signals"].push_back({{"name", name},
                               {"direction", ToString(mapping.signal.direction)},
                               {"property", mapping.property},
                               {"initial", mapping.signal.value}});
  }
  root["tools"] = Json::array();
  for (const auto& [name, tool] : manifest.tools) {
    root["tools"].push_back({{"name", name},
                             {"alias", tool.alias},
                             {"joint_alias", tool.joint_alias},
                             {"tcp_alias", tool.tcp_alias},
                             {"open_aperture", tool.open_aperture},
                             {"closed_aperture", tool.closed_aperture},
                             {"aperture_tolerance", tool.aperture_tolerance},
                             {"open_joint_position", tool.open_joint_position},
                             {"closed_joint_position", tool.closed_joint_position},
                             {"command_property", tool.command_property}});
  }
  return root.dump(2) + "\n";
}

bool IsPoseWithinTolerance(const Pose& observed, const Pose& reference,
                           double translation_tolerance, double rotation_tolerance) {
  const double dx = observed[0] - reference[0];
  const double dy = observed[1] - reference[1];
  const double dz = observed[2] - reference[2];
  const double translation_error = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double quaternion_dot = std::abs(observed[3] * reference[3] + observed[4] * reference[4] +
                                         observed[5] * reference[5] + observed[6] * reference[6]);
  const double rotation_error = 2.0 * std::acos(std::clamp(quaternion_dot, 0.0, 1.0));
  return translation_error <= translation_tolerance && rotation_error <= rotation_tolerance;
}

}  // namespace roborun
