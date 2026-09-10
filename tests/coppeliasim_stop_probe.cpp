#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include "roborun/clock.h"
#include "roborun/coppeliasim_backend.h"
#include "roborun/parser.h"
#include "roborun/runtime.h"

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "usage: roborun_coppeliasim_stop_probe <ur5-model-path>\n";
    return 2;
  }
  std::istringstream input("SERVO_ON\nMOVEJ 0.35 -0.45 0.25 -0.30 0.15 -0.10 TIMEOUT 5000\nSTOP\n");
  const roborun::ParseResult parsed =
      roborun::ParseProgram(input, "lifecycle-control-coppeliasim-stop.task");
  if (!parsed.diagnostics.empty()) {
    std::cerr << "failed to construct probe program\n";
    return 2;
  }

  roborun::CoppeliaSimConfig configuration;
  configuration.model_path = argv[1];
  roborun::CoppeliaSimBackend backend(std::move(configuration));
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  if (session.SubmitProgram(parsed.program).status != roborun::ProgramSubmissionStatus::kAccepted) {
    std::cerr << "failed to submit probe program\n";
    return 2;
  }
  session.Advance();
  session.Advance();
  if (!session.active_command().has_value()) {
    std::cerr << "MOVEJ did not become active before STOP\n";
    return 1;
  }
  session.SubmitControl(roborun::ControlType::kStop, "lifecycle-control-coppeliasim-probe");
  for (int step = 0; step < 200 && !session.is_terminal(); ++step) {
    session.Advance();
  }
  const roborun::ExecutionResult& result = session.result();
  std::cout << "RESULT status="
            << (result.outcome == roborun::ExecutionOutcome::kOperatorStopped ? "passed" : "failed")
            << " outcome=" << roborun::ToString(result.outcome)
            << " lifecycle=" << roborun::ToString(result.lifecycle_state)
            << " final_state=" << roborun::ToString(result.final_state)
            << " finished_business_time_ms=" << result.finished_business_time_ms << '\n';
  return result.outcome == roborun::ExecutionOutcome::kOperatorStopped ? 0 : 1;
}
