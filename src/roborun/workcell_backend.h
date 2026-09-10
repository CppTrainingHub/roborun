#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "roborun/types.h"

namespace roborun {

enum class WorkcellCommandStatus { kAccepted, kRejected };

struct WorkcellCommandSubmission {
  WorkcellCommandStatus status = WorkcellCommandStatus::kRejected;
  std::string message;
  std::optional<BusinessTime> poll_after_ms;
};

struct WorkpieceSnapshot {
  bool present = false;
  bool at_pick = false;
  bool attached = false;
  bool at_place = false;
  int attached_stable_samples = 0;
  int place_stable_samples = 0;
  std::string parent_alias;
  Pose world_pose{};
};

struct WorkcellSnapshot {
  std::uint64_t generation = 0;
  bool valid = false;
  std::string scene_schema;
  JointPositions joint_positions{};
  std::map<std::string, DigitalSignal> signals;
  std::map<std::string, ToolState> tools;
  std::map<std::string, double> tool_openings;
  std::map<std::string, int> tool_stable_samples;
  int required_stable_samples = 1;
  WorkpieceSnapshot workpiece;
  bool grip_failed = false;
  bool placement_failed = false;
  bool collision = false;
  std::vector<std::string> collision_aliases;
  bool simulation_running = false;
  std::string message;
};

class WorkcellBackend {
 public:
  virtual ~WorkcellBackend() = default;
  virtual WorkcellCommandSubmission SubmitToolCommand(const std::string& name,
                                                      ToolPosition position) = 0;
  virtual WorkcellCommandSubmission SetDigitalOutput(const std::string& name, bool value) = 0;
  virtual WorkcellSnapshot PollWorkcell() = 0;
  virtual WorkcellCommandSubmission RequestSafeState(StopReason reason) = 0;
  virtual WorkcellCommandSubmission PollSafeState() = 0;
};

}  // namespace roborun
