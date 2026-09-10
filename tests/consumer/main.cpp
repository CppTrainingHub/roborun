#include <iostream>
#include <sstream>

#include "roborun/mock_robot_backend.h"
#include "roborun/parser.h"
#include "roborun/runtime.h"
#include "roborun/validator.h"

int main() {
  std::istringstream input("SERVO_ON\nMOVEJ 0.20 -0.35 0.30 -0.25 0.15 -0.10\nSTOP\n");
  const roborun::ParseResult parsed = roborun::ParseProgram(input, "consumer.task");
  if (!parsed.diagnostics.empty()) {
    std::cerr << "consumer parse failed\n";
    return 1;
  }

  const roborun::ValidationResult validated = roborun::ValidateProgram(parsed.program);
  if (!validated.diagnostics.empty()) {
    std::cerr << "consumer validation failed\n";
    return 1;
  }

  roborun::MockRobotBackend backend;
  const roborun::RobotTaskRuntime runtime;
  const roborun::ExecutionResult result = runtime.Execute(parsed.program, &backend);
  if (!result.succeeded) {
    std::cerr << "consumer execution failed\n";
    return 1;
  }

  std::cout << "consumer result=passed\n";
  return 0;
}
