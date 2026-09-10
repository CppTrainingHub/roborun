#include "roborun/simulator_lifecycle.h"

#include <exception>

namespace roborun {

std::string ToString(SimulatorLifecycleOperation operation) {
  switch (operation) {
    case SimulatorLifecycleOperation::kStopSimulation:
      return "stop_simulation";
    case SimulatorLifecycleOperation::kDisconnect:
      return "disconnect";
    case SimulatorLifecycleOperation::kQuitSimulator:
      return "quit_simulator";
  }
  return "unknown";
}

SimulatorLifecycleResult RunSimulatorLifecycleOperation(SimulatorLifecycle* lifecycle,
                                                        SimulatorLifecycleOperation operation) {
  try {
    switch (operation) {
      case SimulatorLifecycleOperation::kStopSimulation:
        lifecycle->StopSimulation();
        break;
      case SimulatorLifecycleOperation::kDisconnect:
        lifecycle->Disconnect();
        break;
      case SimulatorLifecycleOperation::kQuitSimulator:
        lifecycle->QuitSimulator();
        break;
    }
    return {operation, true, ""};
  } catch (const std::exception& error) {
    return {operation, false, error.what()};
  }
}

std::vector<SimulatorLifecycleResult> RunSimulatorCleanup(SimulatorLifecycle* lifecycle) {
  std::vector<SimulatorLifecycleResult> results;
  for (const SimulatorLifecycleOperation operation :
       {SimulatorLifecycleOperation::kStopSimulation, SimulatorLifecycleOperation::kDisconnect,
        SimulatorLifecycleOperation::kQuitSimulator}) {
    results.push_back(RunSimulatorLifecycleOperation(lifecycle, operation));
  }
  return results;
}

}  // namespace roborun
