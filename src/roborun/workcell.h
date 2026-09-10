#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "roborun/types.h"

namespace roborun {

struct WorkcellToolConfiguration {
  std::string name;
  std::string alias;
  std::string joint_alias;
  std::string tcp_alias;
  double open_aperture = 0.0;
  double closed_aperture = 0.0;
  double aperture_tolerance = 0.0;
  double open_joint_position = 0.0;
  double closed_joint_position = 0.0;
  std::string command_property;
};

struct WorkcellSignalConfiguration {
  DigitalSignal signal;
  std::string property;
};

struct WorkcellCollisionRule {
  std::string first_alias;
  std::string second_alias;
  bool permitted = false;
};

struct WorkcellObject {
  std::string alias;
  std::string type;
  std::string parent_alias;
  Pose pose{};
};

struct WorkcellConfiguration {
  std::string source;
  std::string manifest_path;
  std::string scene_schema;
  std::string root_alias;
  std::string robot_alias;
  std::string tool_mount_alias;
  std::string tool_alias;
  std::string workpiece_alias;
  std::string pick_alias;
  std::string place_alias;
  Pose workpiece_initial_pose{};
  Pose pick_pose{};
  Pose place_pose{};
  std::map<std::string, WorkcellSignalConfiguration> signals;
  std::map<std::string, WorkcellToolConfiguration> tools;
  std::vector<WorkcellCollisionRule> collisions;
  double pick_radius = 0.06;
  double place_translation_tolerance = 0.02;
  double place_rotation_tolerance = 0.10;
  double joint_tolerance_radians = 0.02;
  int stable_steps = 3;
  BusinessTime default_move_timeout_ms = 1000;
  BusinessTime default_tool_timeout_ms = 500;
  BusinessTime default_grip_timeout_ms = 500;
  BusinessTime default_safe_state_timeout_ms = 500;
};

struct WorkcellManifest {
  int schema_version = 1;
  std::string scene_schema;
  std::vector<WorkcellObject> objects;
  std::map<std::string, WorkcellSignalConfiguration> signals;
  std::map<std::string, WorkcellToolConfiguration> tools;
};

struct WorkcellConfigLoadResult {
  std::optional<WorkcellConfiguration> configuration;
  std::vector<Diagnostic> diagnostics;
};

WorkcellConfiguration MakeDefaultWorkcellConfiguration();
WorkcellConfigLoadResult LoadWorkcellConfiguration(const std::string& path);
WorkcellManifest MakeWorkcellManifest(const WorkcellConfiguration& configuration);
std::vector<Diagnostic> ValidateWorkcellManifest(const WorkcellManifest& manifest,
                                                 const std::string& source);
std::string SerializeWorkcellManifest(const WorkcellManifest& manifest);
bool IsPoseWithinTolerance(const Pose& observed, const Pose& reference,
                           double translation_tolerance, double rotation_tolerance);

}  // namespace roborun
