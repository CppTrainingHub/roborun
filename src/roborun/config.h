#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "roborun/types.h"

namespace roborun {

struct JointLimit {
  std::string name;
  double minimum = 0.0;
  double maximum = 0.0;
};

struct RobotConfiguration {
  std::string robot_id;
  std::array<JointLimit, kJointCount> joints;
  double reference_joint_speed = 0.0;
};

struct PointCatalog {
  struct Point {
    JointPositions joints{};
    std::string joints_path;
  };

  std::string source;
  std::string robot_id;
  std::unordered_map<std::string, Point> points;
};

struct RobotConfigLoadResult {
  std::optional<RobotConfiguration> configuration;
  std::vector<Diagnostic> diagnostics;
};

struct PointCatalogLoadResult {
  std::optional<PointCatalog> catalog;
  std::vector<Diagnostic> diagnostics;
};

struct IOConfiguration {
  std::string source;
  std::map<std::string, DigitalSignal> signals;
};

struct ToolConfiguration {
  std::string source;
  std::map<std::string, ToolState> tools;
};

struct IOConfigLoadResult {
  std::optional<IOConfiguration> configuration;
  std::vector<Diagnostic> diagnostics;
};

struct ToolConfigLoadResult {
  std::optional<ToolConfiguration> configuration;
  std::vector<Diagnostic> diagnostics;
};

RobotConfigLoadResult LoadRobotConfiguration(const std::string& path);
PointCatalogLoadResult LoadPointCatalog(const std::string& path);
IOConfigLoadResult LoadIOConfiguration(const std::string& path);
ToolConfigLoadResult LoadToolConfiguration(const std::string& path);
std::vector<Diagnostic> ValidatePointCatalog(const PointCatalog& catalog,
                                             const RobotConfiguration& robot);
bool IsValidPointName(const std::string& name);

}  // namespace roborun
