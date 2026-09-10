#include <charconv>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "roborun/cli_evidence_barrier.h"
#include "roborun/config.h"
#include "roborun/mock_robot_backend.h"
#include "roborun/parser.h"
#include "roborun/runtime.h"
#include "roborun/scenario.h"
#include "roborun/scripted_mock_backend.h"
#include "roborun/simulator_lifecycle.h"
#include "roborun/validator.h"
#include "roborun/workcell.h"

#ifdef ROBORUN_WITH_COPPELIASIM
#include "roborun/coppeliasim_backend.h"
#endif

namespace {

enum class OutputFormat {
  kHuman,
  kTrace,
};

struct CommandLineOptions {
  std::string backend;
  std::string program_path;
  std::string model_path;
  std::string robot_config_path;
  std::string points_config_path;
  std::string io_config_path;
  std::string tool_config_path;
  std::string scenario_path;
  std::string workcell_config_path;
  std::string resources_path;
  std::string scene_path;
  int coppeliasim_port = 23000;
  std::string evidence_ready_path;
  std::string evidence_continue_path;
  std::string evidence_scenario;
  std::string evidence_scenario_id;
  int evidence_timeout_ms = 0;
  int estop_after_move_polls = 0;
  OutputFormat output_format = OutputFormat::kHuman;
};

void PrintUsage() {
  std::cerr << "Usage: roborun run --backend <mock|coppeliasim> --program <task-file> "
               "[--robot-config <robot.json>] [--points-config <points.json>] "
               "[--io-config <io.json>] [--tool-config <tools.json>] [--scenario <scenario.json>] "
               "[--model <ur5-model>] [--workcell-config <workcell.json>] "
               "[--resources <CoppeliaSim Resources>] [--scene <workcell.ttt>] "
               "[--coppeliasim-port <port>] "
               "[--estop-after-move-polls <count>] "
               "[--evidence-ready <ready.json> --evidence-continue <continue.json> "
               "--evidence-scenario <normal|estop> --evidence-scenario-id <id> "
               "--evidence-timeout-ms <milliseconds>] "
               "[--format <human|trace>]\n";
}

bool ParsePositiveInt(const std::string& value, int* result) {
  const char* begin = value.data();
  const char* end = begin + value.size();
  const auto [position, error] = std::from_chars(begin, end, *result);
  return error == std::errc() && position == end && *result > 0;
}

bool ParseOptions(int argc, char* argv[], CommandLineOptions* options) {
  if (argc < 2 || std::string(argv[1]) != "run") {
    return false;
  }
  for (int index = 2; index < argc; index += 2) {
    if (index + 1 >= argc) {
      return false;
    }
    const std::string flag = argv[index];
    const std::string value = argv[index + 1];
    if (flag == "--backend") {
      options->backend = value;
    } else if (flag == "--program") {
      options->program_path = value;
    } else if (flag == "--model") {
      options->model_path = value;
    } else if (flag == "--robot-config") {
      options->robot_config_path = value;
    } else if (flag == "--points-config") {
      options->points_config_path = value;
    } else if (flag == "--io-config") {
      options->io_config_path = value;
    } else if (flag == "--tool-config") {
      options->tool_config_path = value;
    } else if (flag == "--scenario") {
      options->scenario_path = value;
    } else if (flag == "--workcell-config") {
      options->workcell_config_path = value;
    } else if (flag == "--resources") {
      options->resources_path = value;
    } else if (flag == "--scene") {
      options->scene_path = value;
    } else if (flag == "--coppeliasim-port") {
      if (!ParsePositiveInt(value, &options->coppeliasim_port) ||
          options->coppeliasim_port > 65535) {
        return false;
      }
    } else if (flag == "--evidence-ready") {
      options->evidence_ready_path = value;
    } else if (flag == "--evidence-continue") {
      options->evidence_continue_path = value;
    } else if (flag == "--evidence-scenario") {
      if (value != "normal" && value != "estop") {
        return false;
      }
      options->evidence_scenario = value;
    } else if (flag == "--evidence-scenario-id") {
      if (value.empty() ||
          value.find_first_not_of(
              "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-") !=
              std::string::npos) {
        return false;
      }
      options->evidence_scenario_id = value;
    } else if (flag == "--evidence-timeout-ms") {
      if (!ParsePositiveInt(value, &options->evidence_timeout_ms)) {
        return false;
      }
    } else if (flag == "--estop-after-move-polls") {
      if (!ParsePositiveInt(value, &options->estop_after_move_polls)) {
        return false;
      }
    } else if (flag == "--format") {
      if (value == "human") {
        options->output_format = OutputFormat::kHuman;
      } else if (value == "trace") {
        options->output_format = OutputFormat::kTrace;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  const int evidence_option_count = static_cast<int>(!options->evidence_ready_path.empty()) +
                                    static_cast<int>(!options->evidence_continue_path.empty()) +
                                    static_cast<int>(!options->evidence_scenario.empty()) +
                                    static_cast<int>(!options->evidence_scenario_id.empty()) +
                                    static_cast<int>(options->evidence_timeout_ms > 0);
  const bool evidence_options_valid =
      evidence_option_count == 0 ||
      (evidence_option_count == 5 && options->backend == "coppeliasim" &&
       !options->workcell_config_path.empty() && options->output_format == OutputFormat::kTrace);
  return !options->backend.empty() && !options->program_path.empty() && evidence_options_valid;
}

#ifdef ROBORUN_WITH_COPPELIASIM
bool EvidenceCaptureRequested(const CommandLineOptions& options) {
  return !options.evidence_ready_path.empty();
}
#endif

void PrintDiagnostics(const std::vector<roborun::Diagnostic>& diagnostics) {
  for (const roborun::Diagnostic& diagnostic : diagnostics) {
    std::cerr << "ERROR code=" << roborun::ToString(diagnostic.code)
              << " source=" << std::quoted(diagnostic.source) << " line=" << diagnostic.line
              << " column=" << diagnostic.column << " path=" << std::quoted(diagnostic.path)
              << " message=" << diagnostic.message << '\n';
  }
}

void PrintTraceResult(const roborun::ExecutionResult& result) {
  for (const roborun::TraceEntry& entry : result.trace) {
    std::cout << "TRACE time_ms=" << entry.business_time_ms
              << " command_index=" << entry.command_index << " line=" << entry.line
              << " command=" << roborun::ToString(entry.command)
              << " state_before=" << roborun::ToString(entry.state_before)
              << " state_during=" << roborun::ToString(entry.state_during)
              << " state_after=" << roborun::ToString(entry.state_after)
              << " lifecycle_before=" << roborun::ToString(entry.lifecycle_before)
              << " lifecycle_during=" << roborun::ToString(entry.lifecycle_during)
              << " lifecycle_after=" << roborun::ToString(entry.lifecycle_after)
              << " command_status=" << roborun::ToString(entry.command_status)
              << " control=" << roborun::ToString(entry.control_type)
              << " control_status=" << roborun::ToString(entry.control_status)
              << " control_request_id=" << entry.control_request_id
              << " control_source=" << std::quoted(entry.control_source)
              << " stop_reason=" << roborun::ToString(entry.stop_reason)
              << " stop_status=" << roborun::ToString(entry.stop_status)
              << " outcome=" << roborun::ToString(entry.outcome)
              << " cause=" << std::quoted(entry.event_cause)
              << " point=" << std::quoted(entry.point_name)
              << " signal=" << std::quoted(entry.signal_name)
              << " tool=" << std::quoted(entry.tool_name)
              << " expected=" << (entry.expected_value ? "ON" : "OFF")
              << " observed=" << (entry.observed_value ? "ON" : "OFF")
              << " deadline_ms=" << entry.deadline_ms
              << " commanded_tool=" << roborun::ToString(entry.commanded_tool_position)
              << " observed_tool=" << roborun::ToString(entry.observed_tool_position)
              << " speed_percent=" << entry.speed_percent
              << " backend_steps=" << entry.backend_steps
              << " workcell_generation=" << entry.workcell_generation
              << " targets=" << roborun::FormatJointPositions(entry.targets)
              << " actual=" << roborun::FormatJointPositions(entry.actual_positions) << '\n';
  }
  for (std::size_t index = 0; index < result.alarms.size(); ++index) {
    const roborun::Alarm& alarm = result.alarms[index];
    std::cout << "ALARM index=" << index
              << " primary=" << (result.primary_alarm_index == index ? "true" : "false")
              << " active=" << (alarm.active ? "true" : "false")
              << " code=" << roborun::ToString(alarm.code)
              << " severity=" << roborun::ToString(alarm.severity)
              << " source=" << std::quoted(alarm.source) << " reason=" << std::quoted(alarm.reason)
              << " details=" << std::quoted(alarm.details)
              << " time_ms=" << alarm.raised_business_time_ms
              << " recoverable=" << (alarm.recoverable ? "true" : "false") << '\n';
  }
  if (result.workcell.has_value()) {
    const roborun::WorkcellEvidence& workcell = *result.workcell;
    std::cout << "WORKCELL generation=" << workcell.generation
              << " scene_schema=" << std::quoted(workcell.scene_schema)
              << " workpiece_present=" << (workcell.workpiece_present ? "true" : "false")
              << " workpiece_attached=" << (workcell.workpiece_attached ? "true" : "false")
              << " workpiece_at_place=" << (workcell.workpiece_at_place ? "true" : "false")
              << " collision=" << (workcell.collision ? "true" : "false")
              << " safe_confirmed=" << (result.workcell_safe_confirmed ? "true" : "false")
              << " simulation_running=" << (workcell.simulation_running ? "true" : "false")
              << " joints=" << roborun::FormatJointPositions(workcell.joint_positions) << " pose=[";
    for (std::size_t index = 0; index < workcell.workpiece_pose.size(); ++index) {
      if (index != 0) {
        std::cout << ',';
      }
      std::cout << workcell.workpiece_pose[index];
    }
    std::cout << "]";
    for (const auto& [name, opening] : workcell.tool_openings) {
      std::cout << " tool_" << name << "_opening=" << opening;
    }
    for (const auto& [name, signal] : workcell.signals) {
      std::cout << " signal_" << name << '=' << (signal.value ? "ON" : "OFF");
    }
    for (const std::string& aliases : workcell.collision_aliases) {
      std::cout << " collision_pair=" << std::quoted(aliases);
    }
    std::cout << '\n';
  }
  std::cout << "RESULT status=" << (result.succeeded ? "passed" : "failed")
            << " backend=" << result.backend_name
            << " backend_version=" << std::quoted(result.backend_version)
            << " task=" << std::quoted(result.task_source)
            << " finished_business_time_ms=" << result.finished_business_time_ms
            << " final_state=" << roborun::ToString(result.final_state)
            << " lifecycle=" << roborun::ToString(result.lifecycle_state)
            << " outcome=" << roborun::ToString(result.outcome) << '\n';
}

std::string FormatHumanJointPositions(const roborun::JointPositions& positions) {
  std::ostringstream output;
  output << '[' << std::fixed << std::setprecision(4);
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (index != 0) {
      output << ", ";
    }
    output << positions[index];
  }
  output << ']';
  return output.str();
}

void PrintStateTransition(const roborun::TraceEntry& entry) {
  std::cout << "    state: " << roborun::ToString(entry.state_before);
  if (entry.state_during != entry.state_before) {
    std::cout << " -> " << roborun::ToString(entry.state_during);
  }
  if (entry.state_after != entry.state_during) {
    std::cout << " -> " << roborun::ToString(entry.state_after);
  }
  std::cout << '\n';
}

void PrintHumanResult(const roborun::ExecutionResult& result) {
  std::cout << "RoboRun execution\n"
            << "  task: " << result.task_source << '\n'
            << "  backend: " << result.backend_name << " (" << result.backend_version << ")\n";

  for (const roborun::TraceEntry& entry : result.trace) {
    std::cout << "\n[" << entry.command_index << "] " << roborun::ToString(entry.command);
    if (!entry.point_name.empty()) {
      std::cout << ' ' << entry.point_name;
    }
    std::cout << '\n'
              << "    line: " << entry.line << '\n'
              << "    business time: " << entry.business_time_ms << " ms\n"
              << "    command status: " << roborun::ToString(entry.command_status) << '\n'
              << "    cause: " << entry.event_cause << '\n';
    PrintStateTransition(entry);
    if (entry.command == roborun::CommandType::kMoveJ) {
      std::cout << "    speed: " << entry.speed_percent << "%\n"
                << "    backend steps: " << entry.backend_steps << '\n'
                << "    target joints: " << FormatHumanJointPositions(entry.targets) << '\n'
                << "    actual joints: " << FormatHumanJointPositions(entry.actual_positions)
                << '\n';
    }
    if (entry.command == roborun::CommandType::kSetDo ||
        entry.command == roborun::CommandType::kWaitDi) {
      std::cout << "    signal: " << entry.signal_name
                << " expected: " << (entry.expected_value ? "ON" : "OFF")
                << " observed: " << (entry.observed_value ? "ON" : "OFF")
                << " deadline: " << entry.deadline_ms << " ms\n";
    }
    if (entry.command == roborun::CommandType::kSetTool) {
      std::cout << "    tool: " << entry.tool_name
                << " commanded: " << roborun::ToString(entry.commanded_tool_position)
                << " observed: " << roborun::ToString(entry.observed_tool_position)
                << " deadline: " << entry.deadline_ms << " ms\n";
    }
  }

  std::cout << "\nResult: " << (result.succeeded ? "PASSED" : "FAILED") << '\n'
            << "  final state: " << roborun::ToString(result.final_state) << '\n'
            << "  lifecycle: " << roborun::ToString(result.lifecycle_state) << '\n'
            << "  outcome: " << roborun::ToString(result.outcome) << '\n';
}

void PrintExecutionResult(const roborun::ExecutionResult& result, OutputFormat format) {
  if (format == OutputFormat::kTrace) {
    PrintTraceResult(result);
  } else {
    PrintHumanResult(result);
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string(argv[1]) == "--version") {
    std::cout << "RoboRun " << ROBORUN_VERSION << '\n';
    return 0;
  }

  CommandLineOptions options;
  if (!ParseOptions(argc, argv, &options)) {
    PrintUsage();
    return 2;
  }

  std::vector<roborun::Diagnostic> diagnostics;
  std::optional<roborun::RobotConfiguration> robot_configuration;
  std::optional<roborun::PointCatalog> point_catalog;
  std::optional<roborun::IOConfiguration> io_configuration;
  std::optional<roborun::ToolConfiguration> tool_configuration;
  std::optional<roborun::MockScenario> scenario;
  std::optional<roborun::WorkcellConfiguration> workcell_configuration;
  if (!options.robot_config_path.empty()) {
    roborun::RobotConfigLoadResult loaded =
        roborun::LoadRobotConfiguration(options.robot_config_path);
    diagnostics.insert(diagnostics.end(), loaded.diagnostics.begin(), loaded.diagnostics.end());
    robot_configuration = std::move(loaded.configuration);
  }
  if (!options.points_config_path.empty()) {
    roborun::PointCatalogLoadResult loaded = roborun::LoadPointCatalog(options.points_config_path);
    diagnostics.insert(diagnostics.end(), loaded.diagnostics.begin(), loaded.diagnostics.end());
    point_catalog = std::move(loaded.catalog);
  }
  if (!options.io_config_path.empty()) {
    roborun::IOConfigLoadResult loaded = roborun::LoadIOConfiguration(options.io_config_path);
    diagnostics.insert(diagnostics.end(), loaded.diagnostics.begin(), loaded.diagnostics.end());
    io_configuration = std::move(loaded.configuration);
  }
  if (!options.tool_config_path.empty()) {
    roborun::ToolConfigLoadResult loaded = roborun::LoadToolConfiguration(options.tool_config_path);
    diagnostics.insert(diagnostics.end(), loaded.diagnostics.begin(), loaded.diagnostics.end());
    tool_configuration = std::move(loaded.configuration);
  }
  if (!options.scenario_path.empty()) {
    roborun::ScenarioLoadResult loaded = roborun::LoadMockScenario(options.scenario_path);
    diagnostics.insert(diagnostics.end(), loaded.diagnostics.begin(), loaded.diagnostics.end());
    scenario = std::move(loaded.scenario);
  }
  if (!options.workcell_config_path.empty()) {
    roborun::WorkcellConfigLoadResult loaded =
        roborun::LoadWorkcellConfiguration(options.workcell_config_path);
    diagnostics.insert(diagnostics.end(), loaded.diagnostics.begin(), loaded.diagnostics.end());
    workcell_configuration = std::move(loaded.configuration);
  }
  if (robot_configuration.has_value() && point_catalog.has_value()) {
    const std::vector<roborun::Diagnostic> catalog_diagnostics =
        roborun::ValidatePointCatalog(*point_catalog, *robot_configuration);
    diagnostics.insert(diagnostics.end(), catalog_diagnostics.begin(), catalog_diagnostics.end());
  }

  std::ifstream input(options.program_path);
  roborun::ParseResult parse_result;
  const bool task_opened = static_cast<bool>(input);
  if (!task_opened) {
    diagnostics.push_back(roborun::Diagnostic{roborun::DiagnosticCode::kTaskSyntax,
                                              options.program_path, 0, 0, "",
                                              "cannot open task file"});
  } else {
    parse_result = roborun::ParseProgram(input, options.program_path);
    diagnostics.insert(diagnostics.end(), parse_result.diagnostics.begin(),
                       parse_result.diagnostics.end());
  }
  if (task_opened && parse_result.diagnostics.empty()) {
    const roborun::ValidationResult validation = roborun::ValidateProgram(
        parse_result.program, robot_configuration ? &*robot_configuration : nullptr,
        point_catalog ? &*point_catalog : nullptr, io_configuration ? &*io_configuration : nullptr,
        tool_configuration ? &*tool_configuration : nullptr);
    diagnostics.insert(diagnostics.end(), validation.diagnostics.begin(),
                       validation.diagnostics.end());
  }
  if (!diagnostics.empty()) {
    PrintDiagnostics(diagnostics);
    return 1;
  }

  std::unique_ptr<roborun::RobotBackend> backend;
  roborun::WorkcellBackend* workcell_backend = nullptr;
#ifdef ROBORUN_WITH_COPPELIASIM
  roborun::CoppeliaSimBackend* coppeliasim_backend = nullptr;
#endif
  if (options.backend == "mock") {
    if (scenario.has_value()) {
      backend = std::make_unique<roborun::ScriptedMockBackend>(*scenario);
    } else {
      backend = std::make_unique<roborun::MockRobotBackend>();
    }
  } else if (options.backend == "coppeliasim") {
    if (scenario.has_value()) {
      std::cerr << "ERROR source=" << options.scenario_path
                << " line=0 message=--scenario is only supported by the mock backend\n";
      return 2;
    }
#ifdef ROBORUN_WITH_COPPELIASIM
    roborun::CoppeliaSimConfig configuration;
    configuration.port = options.coppeliasim_port;
    configuration.model_path = options.model_path;
    configuration.resources_path = options.resources_path;
    configuration.scene_path = options.scene_path;
    configuration.workcell = workcell_configuration;
    auto configured_backend =
        std::make_unique<roborun::CoppeliaSimBackend>(std::move(configuration));
    coppeliasim_backend = configured_backend.get();
    if (workcell_configuration.has_value()) {
      workcell_backend = configured_backend.get();
    }
    backend = std::move(configured_backend);
#else
    std::cerr << "ERROR source=" << options.program_path
              << " line=0 message=CoppeliaSim backend was not enabled at build time\n";
    return 2;
#endif
  } else {
    std::cerr << "ERROR source=" << options.program_path << " line=0 message=unknown backend\n";
    return 2;
  }

  roborun::ExecutionResult result;
  bool lifecycle_succeeded = true;
  bool result_printed = false;
#ifdef ROBORUN_WITH_COPPELIASIM
  if (coppeliasim_backend != nullptr) {
    roborun::VirtualClock clock;
    roborun::RuntimeOptions runtime_options;
    runtime_options.disconnect_on_terminal = false;
    if (workcell_configuration.has_value()) {
      runtime_options.move_timeout_ms = workcell_configuration->default_move_timeout_ms;
      runtime_options.tool_timeout_ms = workcell_configuration->default_tool_timeout_ms;
      runtime_options.grip_timeout_ms = workcell_configuration->default_grip_timeout_ms;
      runtime_options.workcell_safe_state_timeout_ms =
          workcell_configuration->default_safe_state_timeout_ms;
    }
    roborun::RuntimeExecution execution(
        backend.get(), &clock, robot_configuration ? &*robot_configuration : nullptr,
        point_catalog ? &*point_catalog : nullptr, io_configuration ? &*io_configuration : nullptr,
        tool_configuration ? &*tool_configuration : nullptr, scenario ? &*scenario : nullptr,
        runtime_options, workcell_backend);
    execution.SubmitProgram(parse_result.program);
    int move_polls = 0;
    bool estop_submitted = false;
    for (int step = 0; !execution.is_terminal() && step < 100000; ++step) {
      execution.Advance();
      if (!estop_submitted && options.estop_after_move_polls > 0 &&
          execution.active_command().has_value() &&
          execution.active_command()->type == roborun::CommandType::kMoveJ &&
          ++move_polls >= options.estop_after_move_polls) {
        execution.SubmitControl(roborun::ControlType::kEstop, "cli-check");
        estop_submitted = true;
      }
    }
    if (!execution.is_terminal()) {
      execution.SubmitControl(roborun::ControlType::kEstop, "one-shot-guard");
      while (!execution.is_terminal()) {
        execution.Advance();
      }
    }
    result = execution.result();
    std::vector<roborun::SimulatorLifecycleResult> cleanup_results;
    if (EvidenceCaptureRequested(options)) {
      PrintExecutionResult(result, options.output_format);
      std::cout << std::flush;
      result_printed = true;
      const roborun::SimulatorLifecycleResult stop = roborun::RunSimulatorLifecycleOperation(
          coppeliasim_backend, roborun::SimulatorLifecycleOperation::kStopSimulation);
      cleanup_results.push_back(stop);
      if (stop.succeeded && result.workcell.has_value()) {
        const roborun::EvidenceBarrierOptions barrier_options{
            options.evidence_ready_path,
            options.evidence_continue_path,
            options.evidence_scenario,
            options.evidence_scenario_id,
            std::chrono::milliseconds(options.evidence_timeout_ms),
            std::chrono::milliseconds(20),
        };
        const roborun::EvidenceBarrierResult barrier = roborun::PublishEvidenceAndWait(
            barrier_options, static_cast<std::uint64_t>(result.workcell->generation));
        if (!barrier.succeeded) {
          lifecycle_succeeded = false;
          std::cerr << "EVIDENCE status=failed phase=capture_wait message="
                    << std::quoted(barrier.message) << '\n';
        }
      } else if (stop.succeeded) {
        lifecycle_succeeded = false;
        std::cerr
            << "EVIDENCE status=failed phase=snapshot message=\"workcell evidence missing\"\n";
      }
      cleanup_results.push_back(roborun::RunSimulatorLifecycleOperation(
          coppeliasim_backend, roborun::SimulatorLifecycleOperation::kDisconnect));
      cleanup_results.push_back(roborun::RunSimulatorLifecycleOperation(
          coppeliasim_backend, roborun::SimulatorLifecycleOperation::kQuitSimulator));
    } else {
      cleanup_results = roborun::RunSimulatorCleanup(coppeliasim_backend);
    }
    for (const roborun::SimulatorLifecycleResult& cleanup : cleanup_results) {
      lifecycle_succeeded = lifecycle_succeeded && cleanup.succeeded;
      std::ostream& output = cleanup.succeeded ? std::cout : std::cerr;
      output << "LIFECYCLE operation=" << roborun::ToString(cleanup.operation)
             << " status=" << (cleanup.succeeded ? "passed" : "failed");
      if (!cleanup.message.empty()) {
        output << " message=" << std::quoted(cleanup.message);
      }
      output << '\n';
    }
  } else
#endif
  {
    const roborun::RobotTaskRuntime runtime;
    result = runtime.Execute(
        parse_result.program, robot_configuration ? &*robot_configuration : nullptr,
        point_catalog ? &*point_catalog : nullptr, io_configuration ? &*io_configuration : nullptr,
        tool_configuration ? &*tool_configuration : nullptr, scenario ? &*scenario : nullptr,
        backend.get(), workcell_backend);
  }
  if (!result_printed) {
    PrintExecutionResult(result, options.output_format);
  }
  if (!result.succeeded || !lifecycle_succeeded) {
    PrintDiagnostics(result.diagnostics);
    return 1;
  }
  return 0;
}
