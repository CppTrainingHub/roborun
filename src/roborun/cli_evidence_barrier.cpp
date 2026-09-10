#include "roborun/cli_evidence_barrier.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <system_error>
#include <thread>

namespace roborun {
namespace {

std::string JsonEscape(const std::string& value) {
  std::ostringstream escaped;
  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped << "\\\\";
        break;
      case '"':
        escaped << "\\\"";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        escaped << character;
        break;
    }
  }
  return escaped.str();
}

std::string MakeReadyRecord(const EvidenceBarrierOptions& options, std::uint64_t generation) {
  std::ostringstream record;
  record << "{\"schema\":\"roborun.gui.evidence-ready.v1\",\"scenario\":\""
         << JsonEscape(options.scenario) << "\",\"scenario_id\":\""
         << JsonEscape(options.scenario_id) << "\",\"generation\":" << generation << "}\n";
  return record.str();
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

}  // namespace

EvidenceBarrierResult PublishEvidenceAndWait(const EvidenceBarrierOptions& options,
                                             std::uint64_t generation) {
  if (options.ready_path.empty() || options.continue_path.empty() || options.scenario.empty() ||
      options.scenario_id.empty() || options.timeout <= std::chrono::milliseconds::zero() ||
      generation == 0) {
    return {false, "invalid evidence barrier options"};
  }
  try {
    if (!options.ready_path.parent_path().empty()) {
      std::filesystem::create_directories(options.ready_path.parent_path());
    }
    if (!options.continue_path.parent_path().empty()) {
      std::filesystem::create_directories(options.continue_path.parent_path());
    }
    std::filesystem::remove(options.ready_path);
    std::filesystem::remove(options.continue_path);
    const std::filesystem::path temporary_path = options.ready_path.string() + ".tmp";
    std::filesystem::remove(temporary_path);
    const std::string record = MakeReadyRecord(options, generation);
    {
      std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
      output << record;
      if (!output) {
        return {false, "failed to write evidence-ready record"};
      }
    }
    std::filesystem::rename(temporary_path, options.ready_path);
    std::cout << "EVIDENCE_READY scenario=" << options.scenario
              << " scenario_id=" << options.scenario_id << " generation=" << generation
              << " ready_file=" << options.ready_path.string() << '\n'
              << std::flush;

    const auto deadline = std::chrono::steady_clock::now() + options.timeout;
    const auto poll_interval = std::max(options.poll_interval, std::chrono::milliseconds(1));
    while (std::chrono::steady_clock::now() < deadline) {
      if (std::filesystem::exists(options.continue_path) &&
          ReadFile(options.continue_path) == record) {
        return {true, ""};
      }
      std::this_thread::sleep_for(poll_interval);
    }
    return {false, "evidence capture timed out"};
  } catch (const std::filesystem::filesystem_error& error) {
    return {false, error.what()};
  }
}

}  // namespace roborun
