#pragma once

#include <string>
#include <vector>

namespace roborun {

enum class SimulatorLifecycleOperation {
  kStopSimulation,
  kDisconnect,
  kQuitSimulator,
};

struct SimulatorLifecycleResult {
  SimulatorLifecycleOperation operation = SimulatorLifecycleOperation::kStopSimulation;
  bool succeeded = false;
  std::string message;
};

class SimulatorLifecycle {
 public:
  virtual ~SimulatorLifecycle() = default;
  virtual void StopSimulation() = 0;
  virtual void Disconnect() = 0;
  virtual void QuitSimulator() = 0;
};

std::string ToString(SimulatorLifecycleOperation operation);
SimulatorLifecycleResult RunSimulatorLifecycleOperation(SimulatorLifecycle* lifecycle,
                                                        SimulatorLifecycleOperation operation);
std::vector<SimulatorLifecycleResult> RunSimulatorCleanup(SimulatorLifecycle* lifecycle);

}  // namespace roborun
