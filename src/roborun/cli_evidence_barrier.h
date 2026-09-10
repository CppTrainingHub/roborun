#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>

namespace roborun {

struct EvidenceBarrierOptions {
  std::filesystem::path ready_path;
  std::filesystem::path continue_path;
  std::string scenario;
  std::string scenario_id;
  std::chrono::milliseconds timeout{0};
  std::chrono::milliseconds poll_interval{10};
};

struct EvidenceBarrierResult {
  bool succeeded = false;
  std::string message;
};

EvidenceBarrierResult PublishEvidenceAndWait(const EvidenceBarrierOptions& options,
                                             std::uint64_t generation);

}  // namespace roborun
