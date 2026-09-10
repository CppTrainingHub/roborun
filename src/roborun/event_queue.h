#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "roborun/types.h"

namespace roborun {

enum class RuntimeEventPriority : std::uint8_t {
  kEmergencyControl = 0,
  kStopControl = 1,
  kLifecycleControl = 2,
  kExternalFeedback = 3,
  kCommandCompletion = 4,
  kTimeout = 5,
};

enum class RuntimeEventType : std::uint8_t {
  kMotionPoll,
  kDelayComplete,
  kDigitalInput,
  kToolFeedback,
  kBackendFault,
  kControl,
  kStopPoll,
};

struct RuntimeEvent {
  BusinessTime time = 0;
  RuntimeEventPriority priority = RuntimeEventPriority::kExternalFeedback;
  std::uint64_t sequence = 0;
  RuntimeEventType type = RuntimeEventType::kMotionPoll;
  std::string name;
  bool value = false;
  std::string detail;
  std::size_t command_index = static_cast<std::size_t>(-1);
  ControlType control_type = ControlType::kNone;
  std::uint64_t control_request_id = 0;
  std::string control_source;
  std::uint64_t command_generation = 0;
  std::uint64_t stop_episode_id = 0;
};

struct EventScheduleRequest {
  BusinessTime time = 0;
  RuntimeEventPriority priority = RuntimeEventPriority::kExternalFeedback;
  RuntimeEventType type = RuntimeEventType::kMotionPoll;
  std::string name;
  bool value = false;
  std::string detail;
  std::size_t command_index = static_cast<std::size_t>(-1);
  std::uint64_t command_generation = 0;
  std::uint64_t stop_episode_id = 0;

  static EventScheduleRequest Simple(BusinessTime time, RuntimeEventPriority priority,
                                     RuntimeEventType type) {
    EventScheduleRequest request;
    request.time = time;
    request.priority = priority;
    request.type = type;
    return request;
  }
};

class EventQueue {
 public:
  void Schedule(EventScheduleRequest request) {
    if (request.time < 0) {
      throw std::invalid_argument("event time cannot be negative");
    }
    events_.push(RuntimeEvent{request.time,
                              request.priority,
                              next_sequence_++,
                              request.type,
                              std::move(request.name),
                              request.value,
                              std::move(request.detail),
                              request.command_index,
                              ControlType::kNone,
                              0,
                              {},
                              request.command_generation,
                              request.stop_episode_id});
  }

  void ScheduleControl(BusinessTime time, ControlType control_type, std::uint64_t request_id,
                       std::string source = {}) {
    if (time < 0) {
      throw std::invalid_argument("event time cannot be negative");
    }
    events_.push(RuntimeEvent{time,
                              PriorityFor(control_type),
                              next_sequence_++,
                              RuntimeEventType::kControl,
                              {},
                              false,
                              {},
                              static_cast<std::size_t>(-1),
                              control_type,
                              request_id,
                              std::move(source),
                              0,
                              0});
  }

  std::optional<RuntimeEvent> PopNext() {
    if (events_.empty()) {
      return std::nullopt;
    }
    RuntimeEvent event = events_.top();
    events_.pop();
    return event;
  }

  std::optional<RuntimeEvent> PeekNext() const {
    if (events_.empty()) {
      return std::nullopt;
    }
    return events_.top();
  }

  bool empty() const { return events_.empty(); }

  void Clear() {
    events_ = {};
    next_sequence_ = 0;
  }

 private:
  static RuntimeEventPriority PriorityFor(ControlType control_type) {
    switch (control_type) {
      case ControlType::kEstop:
        return RuntimeEventPriority::kEmergencyControl;
      case ControlType::kStop:
        return RuntimeEventPriority::kStopControl;
      case ControlType::kPause:
      case ControlType::kResume:
      case ControlType::kReset:
      case ControlType::kNone:
        return RuntimeEventPriority::kLifecycleControl;
    }
    return RuntimeEventPriority::kLifecycleControl;
  }

  struct LaterEventFirst {
    bool operator()(const RuntimeEvent& left, const RuntimeEvent& right) const {
      if (left.time != right.time) {
        return left.time > right.time;
      }
      if (left.priority != right.priority) {
        return left.priority > right.priority;
      }
      return left.sequence > right.sequence;
    }
  };

  std::priority_queue<RuntimeEvent, std::vector<RuntimeEvent>, LaterEventFirst> events_;
  std::uint64_t next_sequence_ = 0;
};

}  // namespace roborun
