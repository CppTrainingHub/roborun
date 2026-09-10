#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "roborun/mock_robot_backend.h"
#include "roborun/runtime.h"
#include "roborun/scripted_mock_backend.h"
#include "roborun/simulator_lifecycle.h"
#include "roborun/workcell.h"
#include "roborun/workcell_backend.h"

namespace roborun {
namespace {

class FakeWorkcellBackend final : public WorkcellBackend {
 public:
  WorkcellSnapshot snapshot;
  int safe_requests = 0;
  int polls = 0;
  int attach_after_polls = 0;
  bool reject_safe_state = false;
  bool freeze_generation_ = false;

  WorkcellCommandSubmission SubmitToolCommand(const std::string& name,
                                              ToolPosition position) override {
    snapshot.tools.at(name).commanded = position;
    snapshot.tools.at(name).observed = position;
    snapshot.tool_stable_samples[name] = snapshot.required_stable_samples;
    return {WorkcellCommandStatus::kAccepted, "", 0};
  }

  WorkcellCommandSubmission SetDigitalOutput(const std::string& name, bool value) override {
    snapshot.signals.at(name).value = value;
    return {WorkcellCommandStatus::kAccepted, "", 0};
  }

  WorkcellSnapshot PollWorkcell() override {
    ++polls;
    if (attach_after_polls > 0 && polls >= attach_after_polls) {
      snapshot.workpiece.attached = true;
      snapshot.workpiece.attached_stable_samples = snapshot.required_stable_samples;
      snapshot.grip_failed = false;
    }
    if (!freeze_generation_) {
      ++snapshot.generation;
    }
    return snapshot;
  }

  WorkcellCommandSubmission RequestSafeState(StopReason reason) override {
    (void)reason;
    ++safe_requests;
    return {reject_safe_state ? WorkcellCommandStatus::kRejected : WorkcellCommandStatus::kAccepted,
            reject_safe_state ? "injected safe-state failure" : "", 0};
  }

  WorkcellCommandSubmission PollSafeState() override {
    return {WorkcellCommandStatus::kAccepted, "", 0};
  }
};

class FailingSimulatorLifecycle final : public SimulatorLifecycle {
 public:
  explicit FailingSimulatorLifecycle(SimulatorLifecycleOperation failure) : failure_(failure) {}

  void StopSimulation() override { Run(SimulatorLifecycleOperation::kStopSimulation); }
  void Disconnect() override { Run(SimulatorLifecycleOperation::kDisconnect); }
  void QuitSimulator() override { Run(SimulatorLifecycleOperation::kQuitSimulator); }

  std::vector<SimulatorLifecycleOperation> calls;

 private:
  void Run(SimulatorLifecycleOperation operation) {
    calls.push_back(operation);
    if (operation == failure_) {
      throw std::runtime_error("injected lifecycle failure");
    }
  }

  SimulatorLifecycleOperation failure_;
};

ExecutionResult ExecuteWithFakeWorkcell(const Program& program, ToolConfiguration* tools,
                                        FakeWorkcellBackend* workcell,
                                        RuntimeOptions options = {}) {
  MockScenario scenario;
  scenario.source = "test";
  scenario.motion_plans.emplace(1, MotionPlan{1, MotionPlanOutcome::kFrozen, 0});
  ScriptedMockBackend robot(scenario);
  VirtualClock clock;
  RuntimeExecution execution(&robot, &clock, nullptr, nullptr, nullptr, tools, nullptr, options,
                             workcell);
  EXPECT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
  while (!execution.is_terminal()) {
    execution.Advance();
  }
  return execution.result();
}

Command MakeCommand(CommandType type, std::size_t line) {
  Command command;
  command.type = type;
  command.line = line;
  return command;
}

WorkcellConfigLoadResult LoadModifiedWorkcellConfig(const std::string& name,
                                                    const std::string& search,
                                                    const std::string& replacement) {
  const std::string source_path =
      std::string(ROBORUN_SOURCE_DIR) + "/examples/config/workcell.json";
  std::ifstream input(source_path);
  std::ostringstream contents;
  contents << input.rdbuf();
  std::string text = contents.str();
  const std::string manifest_line = "  \"manifest_path\": \"workcell_manifest.json\",\n";
  const std::size_t manifest_position = text.find(manifest_line);
  if (manifest_position != std::string::npos) {
    text.erase(manifest_position, manifest_line.size());
  }
  const std::size_t position = text.find(search);
  if (position == std::string::npos) {
    return {{},
            {Diagnostic{DiagnosticCode::kInvalidWorkcellConfiguration, source_path, 0, 0, "",
                        "test fixture replacement target was not found"}}};
  }
  text.replace(position, search.size(), replacement);
  const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
  {
    std::ofstream output(path);
    output << text;
  }
  WorkcellConfigLoadResult loaded = LoadWorkcellConfiguration(path.string());
  std::filesystem::remove(path);
  return loaded;
}

TEST(WorkcellConfigTest, LoadsStableLogicalNamesAndLayout) {
  const std::string path = std::string(ROBORUN_SOURCE_DIR) + "/examples/config/workcell.json";

  const WorkcellConfigLoadResult loaded = LoadWorkcellConfiguration(path);

  ASSERT_TRUE(loaded.configuration.has_value())
      << (loaded.diagnostics.empty() ? "no diagnostic" : loaded.diagnostics.front().message);
  EXPECT_TRUE(loaded.diagnostics.empty());
  EXPECT_EQ(loaded.configuration->scene_schema, "roborun.workcell.v1");
  EXPECT_FALSE(loaded.configuration->manifest_path.empty());
  EXPECT_EQ(loaded.configuration->robot_alias, "/RoboRun/UR5");
  EXPECT_EQ(loaded.configuration->tool_mount_alias, "/RoboRun/UR5/ToolMount");
  EXPECT_EQ(loaded.configuration->tool_alias, "/RoboRun/UR5/ToolMount/RG2");
  EXPECT_EQ(loaded.configuration->workpiece_alias, "/RoboRun/Workpiece");
  EXPECT_EQ(loaded.configuration->pick_alias, "/RoboRun/PICK");
  EXPECT_EQ(loaded.configuration->place_alias, "/RoboRun/PLACE");
  EXPECT_EQ(loaded.configuration->signals.at("PART_READY").signal.direction, SignalDirection::kDi);
  EXPECT_EQ(loaded.configuration->signals.at("CLAMP_ENABLE").signal.direction,
            SignalDirection::kDo);
  EXPECT_EQ(loaded.configuration->signals.at("CLAMP_ENABLE").property,
            "signal.roborun.clamp_enable");
  EXPECT_EQ(loaded.configuration->tools.at("GRIPPER").open_aperture, 0.085);
  EXPECT_EQ(loaded.configuration->tools.at("GRIPPER").joint_alias,
            "/RoboRun/UR5/ToolMount/RG2/Joint");
  EXPECT_EQ(loaded.configuration->stable_steps, 3);
  EXPECT_EQ(loaded.configuration->default_grip_timeout_ms, 500);
  EXPECT_TRUE(loaded.configuration->collisions.empty());
}

TEST(WorkcellConfigTest, RejectsWrongSchemaTypeAndOversizedInteger) {
  const WorkcellConfigLoadResult wrong_schema = LoadModifiedWorkcellConfig(
      "roborun-workcell-wrong-schema.json", "\"schema_version\": 1", "\"schema_version\": \"1\"");
  const WorkcellConfigLoadResult oversized_steps =
      LoadModifiedWorkcellConfig("roborun-workcell-oversized-steps.json", "\"stable_steps\": 3",
                                 "\"stable_steps\": 9223372036854775807");
  const WorkcellConfigLoadResult unsigned_steps =
      LoadModifiedWorkcellConfig("roborun-workcell-unsigned-steps.json", "\"stable_steps\": 3",
                                 "\"stable_steps\": 18446744073709551615");

  ASSERT_FALSE(wrong_schema.diagnostics.empty());
  EXPECT_EQ(wrong_schema.diagnostics.front().code, DiagnosticCode::kWorkcellSchemaMismatch);
  ASSERT_FALSE(oversized_steps.diagnostics.empty());
  EXPECT_EQ(oversized_steps.diagnostics.front().code,
            DiagnosticCode::kInvalidWorkcellConfiguration);
  ASSERT_FALSE(unsigned_steps.diagnostics.empty());
  EXPECT_EQ(unsigned_steps.diagnostics.front().code, DiagnosticCode::kInvalidWorkcellConfiguration);
}

TEST(WorkcellConfigTest, RejectsEmptyToolMappingsWithoutDereferencingThem) {
  const std::string tools = R"("tools": [
    {
      "name": "GRIPPER",
      "alias": "/RoboRun/UR5/ToolMount/RG2",
      "joint_alias": "/RoboRun/UR5/ToolMount/RG2/Joint",
      "tcp_alias": "/RoboRun/UR5/ToolMount/RG2/TCP",
      "open_aperture": 0.085,
      "closed_aperture": 0.001,
      "aperture_tolerance": 0.002,
      "open_joint_position": 0.05,
      "closed_joint_position": -0.045,
      "command_property": "signal.RG2_open"
    }
  ])";
  const WorkcellConfigLoadResult loaded =
      LoadModifiedWorkcellConfig("roborun-workcell-empty-tools.json", tools, "\"tools\": []");

  EXPECT_FALSE(loaded.configuration.has_value());
  ASSERT_FALSE(loaded.diagnostics.empty());
  const auto diagnostic = std::find_if(
      loaded.diagnostics.begin(), loaded.diagnostics.end(), [](const Diagnostic& entry) {
        return entry.code == DiagnosticCode::kInvalidWorkcellConfiguration &&
               entry.path == "/tools";
      });
  EXPECT_NE(diagnostic, loaded.diagnostics.end());
}

TEST(WorkcellConfigTest, ManifestMountsTheToolOnTheRobotFlange) {
  const WorkcellConfiguration configuration = MakeDefaultWorkcellConfiguration();
  const WorkcellManifest manifest = MakeWorkcellManifest(configuration);

  const auto mount = std::find_if(
      manifest.objects.begin(), manifest.objects.end(),
      [&](const WorkcellObject& object) { return object.alias == configuration.tool_mount_alias; });
  const auto tool = std::find_if(
      manifest.objects.begin(), manifest.objects.end(),
      [&](const WorkcellObject& object) { return object.alias == configuration.tool_alias; });

  ASSERT_NE(mount, manifest.objects.end());
  ASSERT_NE(tool, manifest.objects.end());
  EXPECT_EQ(mount->parent_alias, configuration.robot_alias);
  EXPECT_EQ(tool->parent_alias, configuration.tool_mount_alias);
}

TEST(WorkcellConfigTest, DefaultFactoryProducesDeterministicManifest) {
  const WorkcellManifest first = MakeWorkcellManifest(MakeDefaultWorkcellConfiguration());
  const WorkcellManifest second = MakeWorkcellManifest(MakeDefaultWorkcellConfiguration());

  EXPECT_EQ(SerializeWorkcellManifest(first), SerializeWorkcellManifest(second));
}

TEST(WorkcellPoseTest, PlacementRequiresTranslationAndRotationTolerance) {
  const Pose reference{0.25, -0.45, 0.04, 0.0, 0.0, 0.0, 1.0};
  const Pose translated{0.255, -0.45, 0.04, 0.0, 0.0, 0.0, 1.0};
  const Pose rotated{0.25, -0.45, 0.04, 0.0, 0.0, 0.5, 0.866025403784};

  EXPECT_TRUE(IsPoseWithinTolerance(translated, reference, 0.01, 0.1));
  EXPECT_FALSE(IsPoseWithinTolerance(rotated, reference, 0.01, 0.1));
}

TEST(WorkcellConfigTest, RejectsDuplicateLogicalNamesBeforeConnecting) {
  const WorkcellConfiguration configuration = MakeDefaultWorkcellConfiguration();
  WorkcellManifest manifest = MakeWorkcellManifest(configuration);
  manifest.objects.push_back(manifest.objects.front());

  const std::vector<Diagnostic> diagnostics = ValidateWorkcellManifest(manifest, "test");

  ASSERT_FALSE(diagnostics.empty());
  EXPECT_EQ(diagnostics.front().code, DiagnosticCode::kDuplicateWorkcellAlias);
}

TEST(WorkcellConfigTest, ManifestIsStableAndDoesNotExposeSimulatorIdentity) {
  const WorkcellManifest manifest = MakeWorkcellManifest(MakeDefaultWorkcellConfiguration());
  const std::string encoded = SerializeWorkcellManifest(manifest);

  EXPECT_NE(encoded.find("\"schema_version\": 1"), std::string::npos);
  EXPECT_NE(encoded.find("\"scene_schema\": \"roborun.workcell.v1\""), std::string::npos);
  EXPECT_NE(encoded.find("\"alias\": \"/RoboRun/Workpiece\""), std::string::npos);
  EXPECT_EQ(encoded.find("handle"), std::string::npos);
  EXPECT_EQ(encoded.find("process"), std::string::npos);
  EXPECT_EQ(encoded.find("wall_time"), std::string::npos);
}

TEST(WorkcellConfigTest, PublishesVersionedManifestSource) {
  const std::string path =
      std::string(ROBORUN_SOURCE_DIR) + "/examples/config/workcell_manifest.json";
  std::ifstream input(path);
  ASSERT_TRUE(input.is_open());
  std::ostringstream contents;
  contents << input.rdbuf();

  EXPECT_NE(contents.str().find("\"scene_schema\": \"roborun.workcell.v1\""), std::string::npos);
  EXPECT_NE(contents.str().find("\"schema_version\": 1"), std::string::npos);
  EXPECT_NE(contents.str().find("\"alias\": \"/RoboRun/UR5/ToolMount/RG2/TCP\""),
            std::string::npos);
  EXPECT_NE(contents.str().find("\"property\": \"signal.roborun.part_ready\""), std::string::npos);
}

TEST(WorkcellRuntimeTest, RejectsClosedGripperWithoutAnAttachedWorkpiece) {
  ToolConfiguration tools;
  tools.tools.emplace("GRIPPER", ToolState{"GRIPPER", ToolType::kGripper, ToolPosition::kOpen,
                                           ToolPosition::kOpen, 0});
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.snapshot.tools = tools.tools;
  workcell.snapshot.workpiece.present = true;
  workcell.snapshot.workpiece.attached = false;
  workcell.snapshot.grip_failed = true;
  Command close = MakeCommand(CommandType::kSetTool, 2);
  close.timeout_ms = 20;
  close.tool_name = "GRIPPER";
  close.tool_position = ToolPosition::kClosed;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), close, MakeCommand(CommandType::kStop, 3)}};

  const ExecutionResult result = ExecuteWithFakeWorkcell(program, &tools, &workcell);

  ASSERT_FALSE(result.succeeded);
  ASSERT_TRUE(result.primary_alarm_index.has_value());
  EXPECT_EQ(result.alarms[*result.primary_alarm_index].code, DiagnosticCode::kGripFailure);
}

TEST(WorkcellRuntimeTest, ClosedToolWaitsForObservedAttachment) {
  ToolConfiguration tools;
  tools.tools.emplace("GRIPPER", ToolState{"GRIPPER", ToolType::kGripper, ToolPosition::kOpen,
                                           ToolPosition::kOpen, 0});
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.snapshot.scene_schema = "roborun.workcell.v1";
  workcell.snapshot.joint_positions = {0.11, -0.22, 0.33, -0.44, 0.55, -0.66};
  workcell.snapshot.tools = tools.tools;
  workcell.snapshot.signals.emplace("GRIP_OK",
                                    DigitalSignal{"GRIP_OK", SignalDirection::kDi, true, 0});
  workcell.snapshot.workpiece.present = true;
  workcell.snapshot.workpiece.attached = false;
  workcell.snapshot.required_stable_samples = 2;
  workcell.attach_after_polls = 2;
  Command close = MakeCommand(CommandType::kSetTool, 2);
  close.timeout_ms = 30;
  close.tool_name = "GRIPPER";
  close.tool_position = ToolPosition::kClosed;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), close, MakeCommand(CommandType::kStop, 3)}};

  const ExecutionResult result = ExecuteWithFakeWorkcell(program, &tools, &workcell);

  EXPECT_TRUE(result.succeeded);
  ASSERT_TRUE(result.workcell.has_value());
  EXPECT_TRUE(result.workcell->workpiece_attached);
  EXPECT_EQ(result.workcell->scene_schema, "roborun.workcell.v1");
  EXPECT_TRUE(result.workcell->signals.at("GRIP_OK").value);
  EXPECT_EQ(result.workcell->tools.at("GRIPPER").observed, ToolPosition::kClosed);
  EXPECT_EQ(result.workcell->joint_positions, workcell.snapshot.joint_positions);
}

TEST(WorkcellRuntimeTest, UsesConfiguredGripDeadlineWhenTaskOmitsTimeout) {
  ToolConfiguration tools;
  tools.tools.emplace("GRIPPER", ToolState{"GRIPPER", ToolType::kGripper, ToolPosition::kOpen,
                                           ToolPosition::kOpen, 0});
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.snapshot.tools = tools.tools;
  workcell.snapshot.workpiece.present = true;
  Command close = MakeCommand(CommandType::kSetTool, 2);
  close.tool_name = "GRIPPER";
  close.tool_position = ToolPosition::kClosed;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), close, MakeCommand(CommandType::kStop, 3)}};
  RuntimeOptions options;
  options.grip_timeout_ms = 37;

  const ExecutionResult result = ExecuteWithFakeWorkcell(program, &tools, &workcell, options);

  ASSERT_TRUE(result.primary_alarm_index.has_value());
  EXPECT_EQ(result.alarms[*result.primary_alarm_index].code, DiagnosticCode::kToolTimeout);
  EXPECT_EQ(result.finished_business_time_ms, 37);
}

TEST(WorkcellRuntimeTest, ResumeRestartsWorkcellPollingForWaitDi) {
  IOConfiguration signals;
  signals.signals.emplace("PART_READY",
                          DigitalSignal{"PART_READY", SignalDirection::kDi, false, 0});
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.snapshot.signals = signals.signals;
  MockRobotBackend robot;
  VirtualClock clock;
  Command wait = MakeCommand(CommandType::kWaitDi, 2);
  wait.timeout_ms = 30;
  wait.signal_name = "PART_READY";
  wait.signal_value = true;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), wait, MakeCommand(CommandType::kStop, 3)}};
  RuntimeExecution execution(&robot, &clock, nullptr, nullptr, &signals, nullptr, nullptr, {},
                             &workcell);

  ASSERT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
  execution.Advance();
  execution.Advance();
  execution.Advance();
  execution.SubmitControl(ControlType::kPause, "test");
  execution.Advance();
  ASSERT_EQ(execution.lifecycle_state(), LifecycleState::kPaused);
  workcell.snapshot.signals.at("PART_READY").value = true;
  execution.SubmitControl(ControlType::kResume, "test");
  for (int step = 0; !execution.is_terminal() && step < 20; ++step) {
    execution.Advance();
  }

  EXPECT_TRUE(execution.is_terminal());
  EXPECT_TRUE(execution.result().succeeded);
}

TEST(WorkcellRuntimeTest, ResumeToolRequiresFreshStableWorkpieceEvidence) {
  for (ToolPosition target : {ToolPosition::kClosed, ToolPosition::kOpen}) {
    for (bool valid_after_resume : {false, true}) {
      SCOPED_TRACE(ToString(target));
      ToolConfiguration tools;
      tools.tools.emplace("GRIPPER", ToolState{"GRIPPER", ToolType::kGripper, ToolPosition::kOpen,
                                               ToolPosition::kOpen, 0});
      FakeWorkcellBackend workcell;
      workcell.snapshot.valid = true;
      workcell.snapshot.tools = tools.tools;
      workcell.snapshot.required_stable_samples = 3;
      workcell.snapshot.workpiece.present = true;
      MockRobotBackend robot;
      VirtualClock clock;
      Command tool = MakeCommand(CommandType::kSetTool, 2);
      tool.tool_name = "GRIPPER";
      tool.tool_position = target;
      tool.timeout_ms = 40;
      Program program{
          "test",
          {MakeCommand(CommandType::kServoOn, 1), tool, MakeCommand(CommandType::kStop, 3)}};
      RuntimeExecution execution(&robot, &clock, nullptr, nullptr, nullptr, &tools, nullptr, {},
                                 &workcell);
      ASSERT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
      execution.Advance();
      execution.Advance();
      execution.Advance();  // Observe the target position without attachment/place evidence.
      ASSERT_TRUE(execution.active_command().has_value());
      execution.SubmitControl(ControlType::kPause, "test");
      execution.Advance();
      ASSERT_EQ(execution.lifecycle_state(), LifecycleState::kPaused);
      const int polls = workcell.polls;
      execution.SubmitControl(ControlType::kResume, "test");
      execution.Advance();
      ASSERT_TRUE(execution.active_command().has_value());
      EXPECT_EQ(workcell.polls, polls);
      if (valid_after_resume) {
        workcell.snapshot.workpiece.attached = target == ToolPosition::kClosed;
        workcell.snapshot.workpiece.attached_stable_samples = 3;
        workcell.snapshot.workpiece.place_stable_samples = 3;
      }
      for (int step = 0; step < 50 && !execution.is_terminal(); ++step) {
        execution.Advance();
      }
      ASSERT_TRUE(execution.is_terminal());
      EXPECT_EQ(execution.result().succeeded, valid_after_resume);
      EXPECT_GT(workcell.polls, polls);
      if (!valid_after_resume) {
        ASSERT_TRUE(execution.result().primary_alarm_index.has_value());
        EXPECT_EQ(execution.result().alarms[*execution.result().primary_alarm_index].code,
                  DiagnosticCode::kToolTimeout);
      }
    }
  }
}

TEST(WorkcellRuntimeTest, IgnoresRepeatedWorkcellGeneration) {
  IOConfiguration signals;
  signals.signals.emplace("PART_READY",
                          DigitalSignal{"PART_READY", SignalDirection::kDi, false, 0});
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.snapshot.signals = signals.signals;
  MockRobotBackend robot;
  VirtualClock clock;
  Command wait = MakeCommand(CommandType::kWaitDi, 2);
  wait.timeout_ms = 20;
  wait.signal_name = "PART_READY";
  wait.signal_value = true;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), wait, MakeCommand(CommandType::kStop, 3)}};
  RuntimeExecution execution(&robot, &clock, nullptr, nullptr, &signals, nullptr, nullptr, {},
                             &workcell);

  ASSERT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
  execution.Advance();
  execution.Advance();
  execution.Advance();
  workcell.freeze_generation_ = true;
  workcell.snapshot.signals.at("PART_READY").value = true;
  while (!execution.is_terminal()) {
    execution.Advance();
  }

  ASSERT_TRUE(execution.result().primary_alarm_index.has_value());
  EXPECT_EQ(execution.result().alarms[*execution.result().primary_alarm_index].code,
            DiagnosticCode::kDiTimeout);
}

TEST(WorkcellRuntimeTest, RejectsInvalidRepeatedWorkcellGeneration) {
  IOConfiguration signals;
  signals.signals.emplace("PART_READY",
                          DigitalSignal{"PART_READY", SignalDirection::kDi, false, 0});
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.snapshot.signals = signals.signals;
  MockRobotBackend robot;
  VirtualClock clock;
  Command wait = MakeCommand(CommandType::kWaitDi, 2);
  wait.timeout_ms = 20;
  wait.signal_name = "PART_READY";
  wait.signal_value = true;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), wait, MakeCommand(CommandType::kStop, 3)}};
  RuntimeExecution execution(&robot, &clock, nullptr, nullptr, &signals, nullptr, nullptr, {},
                             &workcell);

  ASSERT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
  execution.Advance();
  execution.Advance();
  execution.Advance();
  workcell.freeze_generation_ = true;
  workcell.snapshot.valid = false;
  workcell.snapshot.message = "injected repeated snapshot failure";
  while (!execution.is_terminal()) {
    execution.Advance();
  }

  ASSERT_TRUE(execution.result().primary_alarm_index.has_value());
  const Alarm& alarm = execution.result().alarms[*execution.result().primary_alarm_index];
  EXPECT_EQ(alarm.code, DiagnosticCode::kWorkcellSnapshotFailure);
  EXPECT_EQ(alarm.details, "injected repeated snapshot failure");
}

TEST(WorkcellRuntimeTest, CollisionSnapshotStopsBeforeStartingTheNextCommand) {
  IOConfiguration signals;
  signals.signals.emplace("PART_READY",
                          DigitalSignal{"PART_READY", SignalDirection::kDi, false, 0});
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.snapshot.signals = signals.signals;
  workcell.snapshot.collision = true;
  workcell.snapshot.collision_aliases = {"/RoboRun/UR5|/RoboRun/Obstacle"};
  MockRobotBackend robot;
  VirtualClock clock;
  Command wait = MakeCommand(CommandType::kWaitDi, 2);
  wait.timeout_ms = 20;
  wait.signal_name = "PART_READY";
  wait.signal_value = true;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), wait, MakeCommand(CommandType::kStop, 3)}};
  RuntimeExecution execution(&robot, &clock, nullptr, nullptr, &signals, nullptr, nullptr, {},
                             &workcell);

  ASSERT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
  while (!execution.is_terminal()) {
    execution.Advance();
  }

  ASSERT_TRUE(execution.result().primary_alarm_index.has_value());
  const Alarm& alarm = execution.result().alarms[*execution.result().primary_alarm_index];
  EXPECT_EQ(alarm.code, DiagnosticCode::kCollisionDetected);
  EXPECT_NE(alarm.details.find("generation 1"), std::string::npos);
  EXPECT_NE(alarm.details.find("/RoboRun/UR5|/RoboRun/Obstacle"), std::string::npos);
  const auto trace = std::find_if(
      execution.result().trace.begin(), execution.result().trace.end(),
      [](const TraceEntry& entry) { return entry.event_cause == "workcell_collision"; });
  ASSERT_NE(trace, execution.result().trace.end());
  EXPECT_EQ(trace->workcell_generation, 1U);
  EXPECT_EQ(workcell.safe_requests, 1);
}

TEST(WorkcellRuntimeTest, EstopRequestsWorkcellSafetyExactlyOnce) {
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  MockScenario scenario;
  scenario.source = "test";
  scenario.motion_plans.emplace(1, MotionPlan{1, MotionPlanOutcome::kFrozen, 0});
  ScriptedMockBackend robot(scenario);
  VirtualClock clock;
  Command move = MakeCommand(CommandType::kMoveJ, 2);
  move.movej.targets = {0.2, -0.2, 0.1, -0.1, 0.05, -0.05};
  move.movej.speed_percent = 100;
  move.movej.joint_speed_radians_per_second = 1.0;
  move.timeout_ms = 100;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), move, MakeCommand(CommandType::kStop, 3)}};
  RuntimeExecution execution(&robot, &clock, nullptr, nullptr, nullptr, nullptr, nullptr, {},
                             &workcell);

  ASSERT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
  execution.Advance();
  execution.Advance();
  ASSERT_TRUE(execution.active_command().has_value());
  EXPECT_EQ(execution.SubmitControl(ControlType::kEstop, "test").status,
            ControlSubmissionStatus::kQueued);
  while (!execution.is_terminal()) {
    execution.Advance();
  }

  EXPECT_EQ(execution.result().outcome, ExecutionOutcome::kEmergencyStopped);
  EXPECT_EQ(workcell.safe_requests, 1);
}

TEST(WorkcellRuntimeTest, ResetRequiresFreshCollisionFreeWorkcellRevalidation) {
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.snapshot.scene_schema = "roborun.workcell.v1";
  workcell.snapshot.simulation_running = true;
  MockScenario scenario;
  scenario.source = "test";
  scenario.motion_plans.emplace(1, MotionPlan{1, MotionPlanOutcome::kFrozen, 0});
  ScriptedMockBackend robot(scenario);
  VirtualClock clock;
  RuntimeOptions options;
  options.disconnect_on_terminal = false;
  Command move = MakeCommand(CommandType::kMoveJ, 2);
  move.movej.targets = {0.2, -0.2, 0.1, -0.1, 0.05, -0.05};
  move.movej.speed_percent = 100;
  move.movej.joint_speed_radians_per_second = 1.0;
  move.timeout_ms = 100;
  Program program{
      "test", {MakeCommand(CommandType::kServoOn, 1), move, MakeCommand(CommandType::kStop, 3)}};
  RuntimeExecution execution(&robot, &clock, nullptr, nullptr, nullptr, nullptr, nullptr, options,
                             &workcell);

  ASSERT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
  execution.Advance();
  execution.Advance();
  ASSERT_EQ(execution.SubmitControl(ControlType::kEstop, "test").status,
            ControlSubmissionStatus::kQueued);
  while (!execution.is_terminal()) {
    execution.Advance();
  }
  ASSERT_EQ(execution.lifecycle_state(), LifecycleState::kEmergencyStopped);

  workcell.freeze_generation_ = true;
  execution.SubmitControl(ControlType::kReset, "test");
  execution.Advance();
  EXPECT_EQ(execution.lifecycle_state(), LifecycleState::kEmergencyStopped);

  workcell.freeze_generation_ = false;
  workcell.snapshot.collision = true;
  execution.SubmitControl(ControlType::kReset, "test");
  execution.Advance();
  EXPECT_EQ(execution.lifecycle_state(), LifecycleState::kEmergencyStopped);

  workcell.snapshot.collision = false;
  execution.SubmitControl(ControlType::kReset, "test");
  execution.Advance();
  EXPECT_EQ(execution.lifecycle_state(), LifecycleState::kIdle);
}

TEST(WorkcellLifecycleTest, ReportsEachCleanupFailureAndContinuesRemainingOperations) {
  for (const SimulatorLifecycleOperation failure :
       {SimulatorLifecycleOperation::kStopSimulation, SimulatorLifecycleOperation::kDisconnect,
        SimulatorLifecycleOperation::kQuitSimulator}) {
    FailingSimulatorLifecycle lifecycle(failure);

    const std::vector<SimulatorLifecycleResult> results = RunSimulatorCleanup(&lifecycle);

    ASSERT_EQ(results.size(), 3U);
    ASSERT_EQ(lifecycle.calls.size(), 3U);
    const auto failed =
        std::find_if(results.begin(), results.end(),
                     [](const SimulatorLifecycleResult& result) { return !result.succeeded; });
    ASSERT_NE(failed, results.end());
    EXPECT_EQ(failed->operation, failure);
    EXPECT_EQ(failed->message, "injected lifecycle failure");
  }
}

TEST(WorkcellLifecycleTest, RunsOneRequestedOperationForEvidenceCaptureOrdering) {
  FailingSimulatorLifecycle lifecycle(SimulatorLifecycleOperation::kQuitSimulator);

  const SimulatorLifecycleResult result =
      RunSimulatorLifecycleOperation(&lifecycle, SimulatorLifecycleOperation::kStopSimulation);

  EXPECT_TRUE(result.succeeded);
  ASSERT_EQ(lifecycle.calls.size(), 1U);
  EXPECT_EQ(lifecycle.calls.front(), SimulatorLifecycleOperation::kStopSimulation);
}

TEST(WorkcellLifecycleTest, PreservesRejectedSafeStateAsAWorkcellAlarm) {
  FakeWorkcellBackend workcell;
  workcell.snapshot.valid = true;
  workcell.reject_safe_state = true;
  MockRobotBackend robot;
  VirtualClock clock;
  Program program{"test",
                  {MakeCommand(CommandType::kServoOn, 1), MakeCommand(CommandType::kStop, 2)}};
  RuntimeExecution execution(&robot, &clock, nullptr, nullptr, nullptr, nullptr, nullptr, {},
                             &workcell);
  ASSERT_EQ(execution.SubmitProgram(program).status, ProgramSubmissionStatus::kAccepted);
  execution.Advance();
  ASSERT_EQ(execution.SubmitControl(ControlType::kEstop, "test").status,
            ControlSubmissionStatus::kQueued);
  while (!execution.is_terminal()) {
    execution.Advance();
  }

  const auto alarm = std::find_if(
      execution.result().alarms.begin(), execution.result().alarms.end(), [](const Alarm& entry) {
        return entry.source == "workcell" && entry.reason == "safe_state_rejected";
      });
  EXPECT_NE(alarm, execution.result().alarms.end());
}

}  // namespace
}  // namespace roborun
