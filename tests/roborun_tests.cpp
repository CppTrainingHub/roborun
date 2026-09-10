#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "roborun/clock.h"
#include "roborun/config.h"
#include "roborun/coppeliasim_motion.h"
#include "roborun/event_queue.h"
#include "roborun/mock_robot_backend.h"
#include "roborun/parser.h"
#include "roborun/runtime.h"
#include "roborun/scenario.h"
#include "roborun/scripted_mock_backend.h"
#include "roborun/validator.h"

namespace {

class CountingBackend final : public roborun::RobotBackend {
 public:
  std::string Name() const override {
    ++calls;
    return "counting";
  }
  std::string Version() const override {
    ++calls;
    return "test";
  }
  void Connect() override { ++calls; }
  void SetServoEnabled(bool) override { ++calls; }
  roborun::MotionSubmission SubmitMoveJ(const roborun::MoveJRequest&) override {
    ++calls;
    return {roborun::MotionSubmissionStatus::kAccepted, "", std::nullopt};
  }
  roborun::MotionSnapshot PollMotion() override {
    ++calls;
    return {roborun::MotionStatus::kSucceeded, {}, 0, "", std::nullopt};
  }
  roborun::StopSubmission SubmitStop(roborun::StopReason) override {
    ++calls;
    return {roborun::StopSubmissionStatus::kAccepted, "", 0};
  }
  roborun::StopSnapshot PollStop() override {
    ++calls;
    return {roborun::StopStatus::kStopped, {}, 0, "", std::nullopt};
  }
  void Disconnect() override {
    ++calls;
    ++disconnect_calls;
  }

  mutable int calls = 0;
  int disconnect_calls = 0;
};

class ConnectFailingBackend final : public roborun::RobotBackend {
 public:
  std::string Name() const override { return "connect-failing"; }
  std::string Version() const override { return "test"; }
  void Connect() override {
    ++connect_calls;
    throw std::runtime_error("connection failed after allocating a transport");
  }
  void SetServoEnabled(bool) override {}
  roborun::MotionSubmission SubmitMoveJ(const roborun::MoveJRequest&) override {
    return {roborun::MotionSubmissionStatus::kRejected, "", std::nullopt};
  }
  roborun::MotionSnapshot PollMotion() override {
    return {roborun::MotionStatus::kFailed, {}, 0, "", std::nullopt};
  }
  roborun::StopSubmission SubmitStop(roborun::StopReason) override {
    return {roborun::StopSubmissionStatus::kRejected, "connection failed", std::nullopt};
  }
  roborun::StopSnapshot PollStop() override {
    return {roborun::StopStatus::kFailed, {}, 0, "connection failed", std::nullopt};
  }
  void Disconnect() override { ++disconnect_calls; }

  int connect_calls = 0;
  int disconnect_calls = 0;
};

class ZeroDelayPollingBackend final : public roborun::RobotBackend {
 public:
  std::string Name() const override { return "zero-delay"; }
  std::string Version() const override { return "test"; }
  void Connect() override { connected_ = true; }
  void SetServoEnabled(bool) override {}
  roborun::MotionSubmission SubmitMoveJ(const roborun::MoveJRequest&) override {
    return {roborun::MotionSubmissionStatus::kAccepted, "", 0};
  }
  roborun::MotionSnapshot PollMotion() override {
    return {roborun::MotionStatus::kRunning, {}, 0, "", 0};
  }
  roborun::StopSubmission SubmitStop(roborun::StopReason) override {
    return {roborun::StopSubmissionStatus::kAccepted, "", 0};
  }
  roborun::StopSnapshot PollStop() override {
    return {roborun::StopStatus::kStopping, {}, 0, "", 0};
  }
  void Disconnect() override { connected_ = false; }

 private:
  bool connected_ = false;
};

roborun::ParseResult Parse(const std::string& program) {
  std::istringstream input(program);
  return roborun::ParseProgram(input, "test.task");
}

class ConfigFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    const ::testing::TestInfo* test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    directory_ = std::filesystem::temp_directory_path() /
                 (std::string("roborun-configured-program-") + test_info->test_suite_name() + "-" +
                  test_info->name());
    std::filesystem::create_directories(directory_);
  }

  void TearDown() override { std::filesystem::remove_all(directory_); }

  std::string Write(const std::string& name, const std::string& content) const {
    const std::filesystem::path path = directory_ / name;
    std::ofstream output(path);
    output << content;
    return path.string();
  }

  std::string ValidRobot() const {
    return R"({
      "schema_version": 1,
      "robot_id": "UR5",
      "joints": [
        {"name":"j1","min":-6.2,"max":6.2}, {"name":"j2","min":-6.2,"max":6.2},
        {"name":"j3","min":-6.2,"max":6.2}, {"name":"j4","min":-6.2,"max":6.2},
        {"name":"j5","min":-6.2,"max":6.2}, {"name":"j6","min":-6.2,"max":6.2}
      ],
      "reference_joint_speed": 1.0
    })";
  }

  std::string ValidPoints() const {
    return R"({
      "schema_version": 1,
      "robot_id": "UR5",
      "points": [
        {"name":"HOME","joints":[0.1,-0.2,0.3,-0.4,0.5,-0.6]},
        {"name":"PICK","joints":[0.2,-0.3,0.4,-0.5,0.6,-0.7]},
        {"name":"PLACE","joints":[-0.2,-0.1,0.2,-0.3,0.4,-0.5]}
      ]
    })";
  }

  std::string ValidIo() const {
    return R"({
      "schema_version": 1,
      "signals": [
        {"name":"PART_READY","direction":"DI","initial":false},
        {"name":"CLAMP_ENABLE","direction":"DO","initial":false}
      ]
    })";
  }

  std::string ValidTools() const {
    return R"({
      "schema_version": 1,
      "tools": [
        {"name":"GRIPPER","type":"gripper","commanded_state":"OPEN","feedback_state":"OPEN"}
      ]
    })";
  }

 private:
  std::filesystem::path directory_;
};

TEST(RoboRunLegacyTest, NumericProgramRunsOnMockAtFullSpeed) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ 0.1 -0.2 0.3 -0.4 0.5 -0.6\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::MockRobotBackend backend;
  roborun::RobotTaskRuntime runtime;
  const roborun::ExecutionResult result = runtime.Execute(parsed.program, &backend);

  ASSERT_TRUE(result.succeeded);
  EXPECT_EQ(result.final_state, roborun::RuntimeState::kStopped);
  ASSERT_EQ(result.trace.size(), 4U);
  EXPECT_EQ(result.trace[1].speed_percent, 100);
  EXPECT_TRUE(result.trace[1].point_name.empty());
  EXPECT_EQ(result.trace[1].command_status, roborun::CommandExecutionStatus::kAccepted);
  EXPECT_EQ(result.trace[2].command_status, roborun::CommandExecutionStatus::kSucceeded);
  EXPECT_DOUBLE_EQ(result.trace[2].actual_positions[4], 0.5);
}

TEST(CoppeliaSimMotionTest, StepCountUsesDistanceSpeedAndSimulationTimeStep) {
  const roborun::JointPositions initial{};
  roborun::JointPositions short_target{};
  short_target[0] = 0.125;
  roborun::JointPositions long_target{};
  long_target[0] = 1.0;

  EXPECT_EQ(roborun::ComputeCoppeliaSimMotionSteps(initial, short_target, 1.0, 0.0625), 2);
  EXPECT_EQ(roborun::ComputeCoppeliaSimMotionSteps(initial, long_target, 1.0, 0.0625), 16);
  EXPECT_EQ(roborun::ComputeCoppeliaSimMotionSteps(initial, initial, 1.0, 0.0625), 1);
  EXPECT_THROW(roborun::ComputeCoppeliaSimMotionSteps(initial, long_target, 1.0, 0.0),
               std::runtime_error);
}

TEST(RoboRunLifecycleTest, CallerCanRetainConnectionForExplicitProcessCleanup) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  CountingBackend backend;
  roborun::VirtualClock clock;
  roborun::RuntimeOptions options;
  options.disconnect_on_terminal = false;
  roborun::RuntimeExecution execution(&backend, &clock, nullptr, nullptr, nullptr, nullptr, nullptr,
                                      options);
  ASSERT_EQ(execution.SubmitProgram(parsed.program).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  while (!execution.is_terminal()) {
    execution.Advance();
  }

  EXPECT_TRUE(execution.result().succeeded);
  EXPECT_EQ(backend.disconnect_calls, 0);
}

TEST(RoboRunParserTest, NamedMovePreservesReferenceAndSpeed) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ HOME SPEED 50\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  ASSERT_EQ(parsed.program.commands.size(), 3U);
  EXPECT_EQ(parsed.program.commands[1].point_reference, "HOME");
  EXPECT_EQ(parsed.program.commands[1].movej.speed_percent, 50);
}

TEST(RoboRunParserTest, RejectsMalformedSpeed) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ HOME SPEED fast\nSTOP\n");
  ASSERT_EQ(parsed.diagnostics.size(), 1U);
  EXPECT_EQ(parsed.diagnostics[0].code, roborun::DiagnosticCode::kInvalidSpeed);
  EXPECT_EQ(parsed.diagnostics[0].line, 2U);
}

TEST(RoboRunParserTest, RejectsMissingSpeedForNamedMove) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ HOME\nSTOP\n");
  ASSERT_EQ(parsed.diagnostics.size(), 1U);
  EXPECT_EQ(parsed.diagnostics[0].code, roborun::DiagnosticCode::kInvalidSpeed);
  EXPECT_EQ(parsed.diagnostics[0].line, 2U);
}

TEST(RoboRunParserTest, RejectsInvalidNamedPointReference) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ bad-name SPEED 50\nSTOP\n");
  ASSERT_EQ(parsed.diagnostics.size(), 1U);
  EXPECT_EQ(parsed.diagnostics[0].code, roborun::DiagnosticCode::kInvalidPointName);
  EXPECT_EQ(parsed.diagnostics[0].line, 2U);
}

TEST(RoboRunParserTest, ParsesDelayAndMoveTimeout) {
  const roborun::ParseResult parsed =
      Parse("SERVO_ON\nDELAY 500\nMOVEJ 0 0 0 0 0 0 TIMEOUT 1200\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  ASSERT_EQ(parsed.program.commands.size(), 4U);
  EXPECT_EQ(parsed.program.commands[1].type, roborun::CommandType::kDelay);
  EXPECT_EQ(parsed.program.commands[1].duration_ms, 500);
  ASSERT_TRUE(parsed.program.commands[2].timeout_ms.has_value());
  EXPECT_EQ(*parsed.program.commands[2].timeout_ms, 1200);
}

TEST(RoboRunParserTest, RejectsInvalidDelayDuration) {
  for (const char* source : {"DELAY -1\n", "DELAY 1.5\n", "DELAY 999999999999999999999\n"}) {
    const roborun::ParseResult parsed = Parse(source);
    ASSERT_EQ(parsed.diagnostics.size(), 1U);
    EXPECT_EQ(parsed.diagnostics[0].code, roborun::DiagnosticCode::kInvalidDuration);
  }
}

TEST(RoboRunParserTest, ParsesDigitalIoAndToolCommands) {
  const roborun::ParseResult parsed = Parse(
      "SERVO_ON\nSET_DO CLAMP_ENABLE ON\nWAIT_DI PART_READY ON TIMEOUT 200\n"
      "SET_TOOL GRIPPER CLOSED TIMEOUT 300\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  ASSERT_EQ(parsed.program.commands.size(), 5U);
  EXPECT_EQ(parsed.program.commands[1].type, roborun::CommandType::kSetDo);
  EXPECT_EQ(parsed.program.commands[1].signal_name, "CLAMP_ENABLE");
  EXPECT_TRUE(parsed.program.commands[1].signal_value);
  EXPECT_EQ(parsed.program.commands[2].type, roborun::CommandType::kWaitDi);
  EXPECT_EQ(parsed.program.commands[2].signal_name, "PART_READY");
  EXPECT_EQ(parsed.program.commands[2].timeout_ms, 200);
  EXPECT_EQ(parsed.program.commands[3].type, roborun::CommandType::kSetTool);
  EXPECT_EQ(parsed.program.commands[3].tool_position, roborun::ToolPosition::kClosed);
  EXPECT_EQ(parsed.program.commands[3].timeout_ms, 300);
}

TEST_F(ConfigFixture, LoadsValidRobotAndPointConfigurations) {
  const roborun::RobotConfigLoadResult robot =
      roborun::LoadRobotConfiguration(Write("robot.json", ValidRobot()));
  const roborun::PointCatalogLoadResult points =
      roborun::LoadPointCatalog(Write("points.json", ValidPoints()));

  ASSERT_TRUE(robot.diagnostics.empty());
  ASSERT_TRUE(points.diagnostics.empty());
  ASSERT_TRUE(robot.configuration.has_value());
  ASSERT_TRUE(points.catalog.has_value());
  EXPECT_TRUE(roborun::ValidatePointCatalog(*points.catalog, *robot.configuration).empty());
}

TEST_F(ConfigFixture, LoadsValidIoAndToolConfigurations) {
  const roborun::IOConfigLoadResult io = roborun::LoadIOConfiguration(Write("io.json", ValidIo()));
  const roborun::ToolConfigLoadResult tools =
      roborun::LoadToolConfiguration(Write("tools.json", ValidTools()));

  ASSERT_TRUE(io.diagnostics.empty());
  ASSERT_TRUE(tools.diagnostics.empty());
  ASSERT_TRUE(io.configuration.has_value());
  ASSERT_TRUE(tools.configuration.has_value());
  EXPECT_EQ(io.configuration->signals.at("PART_READY").direction, roborun::SignalDirection::kDi);
  EXPECT_EQ(tools.configuration->tools.at("GRIPPER").observed, roborun::ToolPosition::kOpen);
}

TEST_F(ConfigFixture, RejectsDuplicateAndInvalidIoDefinitions) {
  const roborun::IOConfigLoadResult result =
      roborun::LoadIOConfiguration(Write("invalid-io.json", R"({"schema_version":1,"signals":[
        {"name":"PART_READY","direction":"DI","initial":false},
        {"name":"PART_READY","direction":"DO","initial":true},
        {"name":"bad-name","direction":"XX","initial":false}
      ]})"));
  EXPECT_FALSE(result.configuration.has_value());
  ASSERT_GE(result.diagnostics.size(), 3U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kDuplicateSignal);
  EXPECT_EQ(result.diagnostics[1].code, roborun::DiagnosticCode::kInvalidSignalConfiguration);
}

TEST_F(ConfigFixture, SetDoAndInitiallySatisfiedWaitDiUpdateWorkcellState) {
  const roborun::IOConfigLoadResult io = roborun::LoadIOConfiguration(Write("io.json", ValidIo()));
  ASSERT_TRUE(io.configuration.has_value());
  const roborun::ParseResult parsed =
      Parse("SERVO_ON\nSET_DO CLAMP_ENABLE ON\nWAIT_DI PART_READY OFF TIMEOUT 50\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::MockRobotBackend backend;
  const roborun::ExecutionResult result = roborun::RobotTaskRuntime().Execute(
      parsed.program, nullptr, nullptr, &*io.configuration, nullptr, nullptr, &backend);

  ASSERT_TRUE(result.succeeded);
  EXPECT_EQ(result.finished_business_time_ms, 0);
  EXPECT_TRUE(result.signals.at("CLAMP_ENABLE").value);
  EXPECT_FALSE(result.signals.at("PART_READY").value);
  EXPECT_TRUE(result.trace[1].observed_value);
  EXPECT_EQ(result.trace[3].event_cause, "di_already_satisfied");
}

TEST_F(ConfigFixture, WaitDiUsesScenarioFeedbackAndExactDeadlineWins) {
  const roborun::IOConfigLoadResult io = roborun::LoadIOConfiguration(Write("io.json", ValidIo()));
  const roborun::ScenarioLoadResult scenario =
      roborun::LoadMockScenario(Write("scenario.json", R"({"schema_version":1,"events":[
        {"time_ms":200,"kind":"DI","name":"PART_READY","value":true}
      ]})"));
  ASSERT_TRUE(io.configuration.has_value());
  ASSERT_TRUE(scenario.scenario.has_value());
  const roborun::ParseResult parsed = Parse("SERVO_ON\nWAIT_DI PART_READY ON TIMEOUT 200\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::ScriptedMockBackend backend(*scenario.scenario);
  const roborun::ExecutionResult result = roborun::RobotTaskRuntime().Execute(
      parsed.program, nullptr, nullptr, &*io.configuration, nullptr, &*scenario.scenario, &backend);

  ASSERT_TRUE(result.succeeded);
  EXPECT_EQ(result.finished_business_time_ms, 200);
  EXPECT_TRUE(result.signals.at("PART_READY").value);
  EXPECT_EQ(result.diagnostics.size(), 0U);
}

TEST_F(ConfigFixture, CurrentTimeScenarioFeedbackPrecedesCommandStart) {
  const roborun::IOConfigLoadResult io = roborun::LoadIOConfiguration(Write("io.json", ValidIo()));
  const roborun::ScenarioLoadResult scenario =
      roborun::LoadMockScenario(Write("scenario.json", R"({"schema_version":1,"events":[
        {"time_ms":0,"kind":"DI","name":"PART_READY","value":true}
      ]})"));
  ASSERT_TRUE(io.configuration.has_value());
  ASSERT_TRUE(scenario.scenario.has_value());
  const roborun::ParseResult parsed = Parse("SERVO_ON\nWAIT_DI PART_READY ON TIMEOUT 20\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::ScriptedMockBackend backend(*scenario.scenario);
  const roborun::ExecutionResult result = roborun::RobotTaskRuntime().Execute(
      parsed.program, nullptr, nullptr, &*io.configuration, nullptr, &*scenario.scenario, &backend);

  ASSERT_TRUE(result.succeeded);
  ASSERT_EQ(result.trace.size(), 4U);
  EXPECT_EQ(result.trace[2].event_cause, "di_already_satisfied");
}

TEST_F(ConfigFixture, SameTimestampFeedbackPrecedesFollowingToolCommand) {
  const roborun::IOConfigLoadResult io = roborun::LoadIOConfiguration(Write("io.json", ValidIo()));
  const roborun::ToolConfigLoadResult tools =
      roborun::LoadToolConfiguration(Write("tools.json", ValidTools()));
  const roborun::ScenarioLoadResult scenario =
      roborun::LoadMockScenario(Write("scenario.json", R"({"schema_version":1,"events":[
        {"time_ms":50,"kind":"DI","name":"PART_READY","value":true},
        {"time_ms":50,"kind":"TOOL","name":"GRIPPER","state":"CLOSED"}
      ]})"));
  ASSERT_TRUE(io.configuration.has_value());
  ASSERT_TRUE(tools.configuration.has_value());
  ASSERT_TRUE(scenario.scenario.has_value());
  const roborun::ParseResult parsed = Parse(
      "SERVO_ON\nWAIT_DI PART_READY ON TIMEOUT 50\nSET_TOOL GRIPPER CLOSED TIMEOUT 10\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::ScriptedMockBackend backend(*scenario.scenario);
  const roborun::ExecutionResult result =
      roborun::RobotTaskRuntime().Execute(parsed.program, nullptr, nullptr, &*io.configuration,
                                          &*tools.configuration, &*scenario.scenario, &backend);

  ASSERT_TRUE(result.succeeded);
  ASSERT_EQ(result.trace.size(), 6U);
  EXPECT_EQ(result.trace[4].event_cause, "tool_already_satisfied");
}

TEST_F(ConfigFixture, WaitDiTimeoutRequestsStopAndPreservesFailureCode) {
  const roborun::IOConfigLoadResult io = roborun::LoadIOConfiguration(Write("io.json", ValidIo()));
  ASSERT_TRUE(io.configuration.has_value());
  const roborun::ParseResult parsed = Parse("SERVO_ON\nWAIT_DI PART_READY ON TIMEOUT 20\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::ScriptedMockBackend backend(roborun::MockScenario{});
  const roborun::ExecutionResult result = roborun::RobotTaskRuntime().Execute(
      parsed.program, nullptr, nullptr, &*io.configuration, nullptr, nullptr, &backend);

  ASSERT_FALSE(result.succeeded);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kDiTimeout);
  EXPECT_EQ(backend.stop_requests(), 1);
  EXPECT_EQ(result.finished_business_time_ms, 20);
  EXPECT_EQ(result.trace.back().command_status, roborun::CommandExecutionStatus::kTimedOut);
}

TEST_F(ConfigFixture, SetToolWaitsForFeedbackAndTimesOutDeterministically) {
  const roborun::ToolConfigLoadResult tools =
      roborun::LoadToolConfiguration(Write("tools.json", ValidTools()));
  const roborun::ScenarioLoadResult scenario =
      roborun::LoadMockScenario(Write("scenario.json", R"({"schema_version":1,"events":[
        {"time_ms":100,"kind":"TOOL","name":"GRIPPER","state":"CLOSED"}
      ]})"));
  ASSERT_TRUE(tools.configuration.has_value());
  ASSERT_TRUE(scenario.scenario.has_value());

  const roborun::ParseResult parsed =
      Parse("SERVO_ON\nSET_TOOL GRIPPER CLOSED TIMEOUT 100\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  roborun::ScriptedMockBackend backend(*scenario.scenario);
  const roborun::ExecutionResult result =
      roborun::RobotTaskRuntime().Execute(parsed.program, nullptr, nullptr, nullptr,
                                          &*tools.configuration, &*scenario.scenario, &backend);
  ASSERT_TRUE(result.succeeded);
  EXPECT_EQ(result.finished_business_time_ms, 100);
  EXPECT_EQ(result.tools.at("GRIPPER").commanded, roborun::ToolPosition::kClosed);
  EXPECT_EQ(result.tools.at("GRIPPER").observed, roborun::ToolPosition::kClosed);

  const roborun::ParseResult timeout_program =
      Parse("SERVO_ON\nSET_TOOL GRIPPER CLOSED TIMEOUT 10\nSTOP\n");
  roborun::ScriptedMockBackend timeout_backend(roborun::MockScenario{});
  const roborun::ExecutionResult timeout =
      roborun::RobotTaskRuntime().Execute(timeout_program.program, nullptr, nullptr, nullptr,
                                          &*tools.configuration, nullptr, &timeout_backend);
  ASSERT_FALSE(timeout.succeeded);
  ASSERT_EQ(timeout.diagnostics.size(), 1U);
  EXPECT_EQ(timeout.diagnostics[0].code, roborun::DiagnosticCode::kToolTimeout);
  EXPECT_EQ(timeout_backend.stop_requests(), 1);
}

TEST_F(ConfigFixture, ScriptedScenarioCoversDelayedRejectFrozenAndConnectionLoss) {
  const roborun::ScenarioLoadResult delayed_scenario =
      roborun::LoadMockScenario(Write("delayed.json", R"({"schema_version":1,"motions":[
        {"movej":1,"outcome":"delayed_success","delay_ms":75}
      ]})"));
  ASSERT_TRUE(delayed_scenario.scenario.has_value());
  const roborun::ParseResult delayed_program =
      Parse("SERVO_ON\nMOVEJ 0 0 0 0 0 0 TIMEOUT 100\nSTOP\n");
  roborun::ScriptedMockBackend delayed_backend(*delayed_scenario.scenario);
  const roborun::ExecutionResult delayed =
      roborun::RobotTaskRuntime().Execute(delayed_program.program, &delayed_backend);
  ASSERT_TRUE(delayed.succeeded);
  EXPECT_EQ(delayed.finished_business_time_ms, 75);

  const roborun::ScenarioLoadResult rejected_scenario =
      roborun::LoadMockScenario(Write("rejected.json", R"({"schema_version":1,"motions":[
        {"movej":1,"outcome":"rejected"}
      ]})"));
  ASSERT_TRUE(rejected_scenario.scenario.has_value());
  roborun::ScriptedMockBackend rejected_backend(*rejected_scenario.scenario);
  const roborun::ExecutionResult rejected =
      roborun::RobotTaskRuntime().Execute(delayed_program.program, &rejected_backend);
  ASSERT_FALSE(rejected.succeeded);
  ASSERT_EQ(rejected.diagnostics.size(), 1U);
  EXPECT_EQ(rejected.diagnostics[0].code, roborun::DiagnosticCode::kMotionRejected);
  EXPECT_EQ(rejected_backend.stop_requests(), 1);

  const roborun::ScenarioLoadResult frozen_scenario =
      roborun::LoadMockScenario(Write("frozen.json", R"({"schema_version":1,"motions":[
        {"movej":1,"outcome":"frozen"}
      ]})"));
  ASSERT_TRUE(frozen_scenario.scenario.has_value());
  const roborun::ParseResult frozen_program =
      Parse("SERVO_ON\nMOVEJ 0 0 0 0 0 0 TIMEOUT 40\nSTOP\n");
  roborun::ScriptedMockBackend frozen_backend(*frozen_scenario.scenario);
  const roborun::ExecutionResult frozen =
      roborun::RobotTaskRuntime().Execute(frozen_program.program, &frozen_backend);
  ASSERT_FALSE(frozen.succeeded);
  ASSERT_EQ(frozen.diagnostics.size(), 1U);
  EXPECT_EQ(frozen.diagnostics[0].code, roborun::DiagnosticCode::kMotionTimeout);
  EXPECT_EQ(frozen.finished_business_time_ms, 40);
  EXPECT_EQ(frozen_backend.stop_requests(), 1);
  EXPECT_NE(std::find(frozen_backend.call_log().begin(), frozen_backend.call_log().end(),
                      "submit_stop:timeout"),
            frozen_backend.call_log().end());

  const roborun::ScenarioLoadResult connection_loss_scenario =
      roborun::LoadMockScenario(Write("connection-loss.json", R"({"schema_version":1,"motions":[
        {"movej":1,"outcome":"delayed_success","delay_ms":75}
      ],"events":[
        {"time_ms":15,"kind":"CONNECTION_LOSS"}
      ]})"));
  ASSERT_TRUE(connection_loss_scenario.scenario.has_value());
  roborun::ScriptedMockBackend connection_loss_backend(*connection_loss_scenario.scenario);
  const roborun::ExecutionResult connection_loss = roborun::RobotTaskRuntime().Execute(
      frozen_program.program, nullptr, nullptr, nullptr, nullptr,
      &*connection_loss_scenario.scenario, &connection_loss_backend);
  ASSERT_FALSE(connection_loss.succeeded);
  ASSERT_EQ(connection_loss.diagnostics.size(), 1U);
  EXPECT_EQ(connection_loss.diagnostics[0].code, roborun::DiagnosticCode::kBackendConnectionLoss);
  EXPECT_EQ(connection_loss.finished_business_time_ms, 15);
}

TEST_F(ConfigFixture, InvalidScenarioFailsBeforeBackendConnection) {
  const roborun::ScenarioLoadResult invalid =
      roborun::LoadMockScenario(Write("invalid-scenario.json", R"({"schema_version":1,"motions":[
        {"movej":2,"outcome":"immediate_success"}
      ],"events":[
        {"time_ms":20,"kind":"DI","name":"UNKNOWN","value":true},
        {"time_ms":20,"kind":"CONNECTION_LOSS"}
      ]})"));
  ASSERT_TRUE(invalid.scenario.has_value());
  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n");
  CountingBackend backend;
  const roborun::ExecutionResult result = roborun::RobotTaskRuntime().Execute(
      parsed.program, nullptr, nullptr, nullptr, nullptr, &*invalid.scenario, &backend);
  EXPECT_FALSE(result.succeeded);
  EXPECT_EQ(backend.calls, 0);
  ASSERT_FALSE(result.diagnostics.empty());
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kInvalidScenario);
}

TEST_F(ConfigFixture, RejectsConnectionLossBeforeBackendConnection) {
  const roborun::ScenarioLoadResult scenario = roborun::LoadMockScenario(
      Write("connection-loss-at-start.json", R"({"schema_version":1,"events":[
        {"time_ms":0,"kind":"CONNECTION_LOSS"}
      ]})"));
  ASSERT_TRUE(scenario.scenario.has_value());
  const roborun::ParseResult parsed = Parse("SERVO_ON\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  CountingBackend backend;

  const roborun::ExecutionResult result = roborun::RobotTaskRuntime().Execute(
      parsed.program, nullptr, nullptr, nullptr, nullptr, &*scenario.scenario, &backend);

  ASSERT_FALSE(result.succeeded);
  EXPECT_EQ(backend.calls, 0);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kInvalidScenario);
}

TEST(RoboRunRuntimeTest, CleansUpBackendAfterConnectionFailure) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  ConnectFailingBackend backend;

  const roborun::ExecutionResult result =
      roborun::RobotTaskRuntime().Execute(parsed.program, &backend);

  ASSERT_FALSE(result.succeeded);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kBackendFailure);
  EXPECT_EQ(backend.connect_calls, 1);
  EXPECT_EQ(backend.disconnect_calls, 1);
}

TEST_F(ConfigFixture, RejectsIrrelevantScenarioEventFields) {
  const roborun::ScenarioLoadResult scenario = roborun::LoadMockScenario(
      Write("invalid-fields.scenario.json", R"({"schema_version":1,"events":[
        {"time_ms":1,"kind":"CONNECTION_LOSS","name":"unexpected"}
      ]})"));

  EXPECT_FALSE(scenario.scenario.has_value());
  ASSERT_EQ(scenario.diagnostics.size(), 1U);
  EXPECT_EQ(scenario.diagnostics[0].code, roborun::DiagnosticCode::kInvalidScenario);
  EXPECT_EQ(scenario.diagnostics[0].path, "/events/0/name");
}

TEST_F(ConfigFixture, ReportsMalformedJsonWithSourcePosition) {
  const std::string path = Write("broken.json", "{\n  \"schema_version\":\n");
  const roborun::RobotConfigLoadResult result = roborun::LoadRobotConfiguration(path);

  ASSERT_EQ(result.diagnostics.size(), 1U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kJsonSyntax);
  EXPECT_EQ(result.diagnostics[0].source, path);
  EXPECT_GT(result.diagnostics[0].line, 0U);
  EXPECT_GT(result.diagnostics[0].column, 0U);
}

TEST_F(ConfigFixture, ReportsMissingFileAndInvalidRobotConstraints) {
  const roborun::RobotConfigLoadResult missing =
      roborun::LoadRobotConfiguration((std::filesystem::path("missing") / "robot.json").string());
  ASSERT_EQ(missing.diagnostics.size(), 1U);
  EXPECT_EQ(missing.diagnostics[0].code, roborun::DiagnosticCode::kConfigIo);

  const roborun::RobotConfigLoadResult invalid = roborun::LoadRobotConfiguration(
      Write("invalid-robot.json", R"({"schema_version":2,"robot_id":"UR5","joints":[
        {"name":"j1","min":1,"max":0}, {"name":"j2","min":-1,"max":1},
        {"name":"j3","min":-1,"max":1}, {"name":"j4","min":-1,"max":1},
        {"name":"j5","min":-1,"max":1}, {"name":"j6","min":-1,"max":1}
      ],"reference_joint_speed":0})"));
  EXPECT_FALSE(invalid.configuration.has_value());
  ASSERT_EQ(invalid.diagnostics.size(), 3U);
  EXPECT_EQ(invalid.diagnostics[0].code, roborun::DiagnosticCode::kSchemaMismatch);
  EXPECT_EQ(invalid.diagnostics[1].code, roborun::DiagnosticCode::kJointLimitViolation);
  EXPECT_EQ(invalid.diagnostics[2].code, roborun::DiagnosticCode::kInvalidConfiguration);
}

TEST_F(ConfigFixture, ReportsWrongConfigurationFieldTypeWithPath) {
  std::string robot = ValidRobot();
  const std::size_t speed = robot.find("\"reference_joint_speed\": 1.0");
  ASSERT_NE(speed, std::string::npos);
  robot.replace(speed, std::string("\"reference_joint_speed\": 1.0").size(),
                "\"reference_joint_speed\": \"fast\"");

  const roborun::RobotConfigLoadResult result =
      roborun::LoadRobotConfiguration(Write("wrong-type.json", robot));

  ASSERT_EQ(result.diagnostics.size(), 1U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kInvalidConfiguration);
  EXPECT_EQ(result.diagnostics[0].path, "/reference_joint_speed");
}

TEST_F(ConfigFixture, RejectsNonFiniteJointInput) {
  std::string points = ValidPoints();
  const std::size_t joint = points.find("0.1");
  ASSERT_NE(joint, std::string::npos);
  points.replace(joint, std::string("0.1").size(), "1e999");

  const roborun::PointCatalogLoadResult result =
      roborun::LoadPointCatalog(Write("nonfinite-joint.json", points));

  ASSERT_EQ(result.diagnostics.size(), 1U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kJsonSyntax);
  EXPECT_GT(result.diagnostics[0].line, 0U);
  EXPECT_GT(result.diagnostics[0].column, 0U);
}

TEST_F(ConfigFixture, RejectsSchemaVersionOutsideIntegerRange) {
  const roborun::RobotConfigLoadResult result = roborun::LoadRobotConfiguration(
      Write("overflow-schema.json", R"({"schema_version":4294967297,"robot_id":"UR5","joints":[
        {"name":"j1","min":-1,"max":1}, {"name":"j2","min":-1,"max":1},
        {"name":"j3","min":-1,"max":1}, {"name":"j4","min":-1,"max":1},
        {"name":"j5","min":-1,"max":1}, {"name":"j6","min":-1,"max":1}
      ],"reference_joint_speed":1})"));
  ASSERT_FALSE(result.diagnostics.empty());
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kSchemaMismatch);
}

TEST_F(ConfigFixture, RejectsInvalidPointCatalogEntries) {
  const roborun::PointCatalogLoadResult result = roborun::LoadPointCatalog(
      Write("points.json", R"({"schema_version":1,"robot_id":"UR5","points":[
        {"name":"HOME","joints":[0,0,0,0,0,0]},
        {"name":"HOME","joints":[0,0,0,0,0,0]},
        {"name":"bad-name","joints":[0,0,0,0,0]},
        {"name":"TOO_HIGH","joints":[0,0,0,0,0,0]}
      ]})"));

  EXPECT_FALSE(result.catalog.has_value());
  ASSERT_GE(result.diagnostics.size(), 2U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kDuplicatePoint);
  EXPECT_EQ(result.diagnostics[1].code, roborun::DiagnosticCode::kInvalidPointName);
}

TEST_F(ConfigFixture, NamedProgramResolvesAndRunsOnMock) {
  std::string robot_json = ValidRobot();
  const std::size_t reference_speed = robot_json.find("\"reference_joint_speed\": 1.0");
  ASSERT_NE(reference_speed, std::string::npos);
  robot_json.replace(reference_speed, std::string("\"reference_joint_speed\": 1.0").size(),
                     "\"reference_joint_speed\": 2.0");
  const roborun::RobotConfigLoadResult robot =
      roborun::LoadRobotConfiguration(Write("robot.json", robot_json));
  const roborun::PointCatalogLoadResult points =
      roborun::LoadPointCatalog(Write("points.json", ValidPoints()));
  ASSERT_TRUE(robot.configuration.has_value());
  ASSERT_TRUE(points.catalog.has_value());
  ASSERT_TRUE(roborun::ValidatePointCatalog(*points.catalog, *robot.configuration).empty());

  const roborun::ParseResult parsed =
      Parse("SERVO_ON\nMOVEJ HOME SPEED 100\nMOVEJ PICK SPEED 50\nMOVEJ PLACE SPEED 25\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  roborun::MockRobotBackend backend;
  roborun::RobotTaskRuntime runtime;
  const roborun::ExecutionResult result =
      runtime.Execute(parsed.program, &*robot.configuration, &*points.catalog, &backend);

  ASSERT_TRUE(result.succeeded);
  ASSERT_EQ(result.trace.size(), 8U);
  EXPECT_EQ(result.trace[1].point_name, "HOME");
  EXPECT_EQ(result.trace[3].speed_percent, 50);
  EXPECT_EQ(result.trace[5].point_name, "PLACE");
  EXPECT_EQ(backend.last_request().point_name, "PLACE");
  EXPECT_EQ(backend.last_request().speed_percent, 25);
  EXPECT_DOUBLE_EQ(backend.last_request().joint_speed_radians_per_second, 0.5);
  EXPECT_DOUBLE_EQ(result.trace[6].actual_positions[0], -0.2);
}

TEST_F(ConfigFixture, InvalidNamedProgramDoesNotCallBackend) {
  const roborun::RobotConfigLoadResult robot =
      roborun::LoadRobotConfiguration(Write("robot.json", ValidRobot()));
  const roborun::PointCatalogLoadResult points =
      roborun::LoadPointCatalog(Write("points.json", ValidPoints()));
  ASSERT_TRUE(robot.configuration.has_value());
  ASSERT_TRUE(points.catalog.has_value());

  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ MISSING SPEED 101\nSTOP\n");
  CountingBackend backend;
  roborun::RobotTaskRuntime runtime;
  const roborun::ExecutionResult result =
      runtime.Execute(parsed.program, &*robot.configuration, &*points.catalog, &backend);

  EXPECT_FALSE(result.succeeded);
  EXPECT_EQ(backend.calls, 0);
  ASSERT_EQ(result.diagnostics.size(), 2U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kInvalidSpeed);
  EXPECT_EQ(result.diagnostics[1].code, roborun::DiagnosticCode::kUnknownPoint);
}

TEST_F(ConfigFixture, RejectsRobotMismatchAndOutOfLimitPoint) {
  const roborun::RobotConfigLoadResult robot =
      roborun::LoadRobotConfiguration(Write("robot.json", ValidRobot()));
  const roborun::PointCatalogLoadResult points = roborun::LoadPointCatalog(
      Write("points.json", R"({"schema_version":1,"robot_id":"OTHER","points":[
        {"name":"HOME","joints":[7,0,0,0,0,0]}
      ]})"));
  ASSERT_TRUE(robot.configuration.has_value());
  ASSERT_TRUE(points.catalog.has_value());

  const std::vector<roborun::Diagnostic> diagnostics =
      roborun::ValidatePointCatalog(*points.catalog, *robot.configuration);
  ASSERT_EQ(diagnostics.size(), 2U);
  EXPECT_EQ(diagnostics[0].code, roborun::DiagnosticCode::kRobotMismatch);
  EXPECT_EQ(diagnostics[1].code, roborun::DiagnosticCode::kJointLimitViolation);
  EXPECT_EQ(diagnostics[1].path, "/points/0/joints/0");
}

TEST_F(ConfigFixture, RuntimeRejectsMismatchedPointCatalogBeforeBackendCalls) {
  const roborun::RobotConfigLoadResult robot =
      roborun::LoadRobotConfiguration(Write("robot.json", ValidRobot()));
  const roborun::PointCatalogLoadResult points = roborun::LoadPointCatalog(
      Write("points.json", R"({"schema_version":1,"robot_id":"OTHER","points":[
        {"name":"UNUSED","joints":[7,0,0,0,0,0]}
      ]})"));
  ASSERT_TRUE(robot.configuration.has_value());
  ASSERT_TRUE(points.catalog.has_value());
  const roborun::ParseResult parsed = Parse("SERVO_ON\nSTOP\n");
  CountingBackend backend;

  const roborun::ExecutionResult result = roborun::RobotTaskRuntime().Execute(
      parsed.program, &*robot.configuration, &*points.catalog, &backend);

  EXPECT_FALSE(result.succeeded);
  EXPECT_EQ(backend.calls, 0);
  ASSERT_EQ(result.diagnostics.size(), 2U);
  EXPECT_EQ(result.diagnostics[0].code, roborun::DiagnosticCode::kRobotMismatch);
  EXPECT_EQ(result.diagnostics[1].code, roborun::DiagnosticCode::kJointLimitViolation);
}

TEST_F(ConfigFixture, RejectsUnknownPointAndOutOfRangeSpeed) {
  const roborun::RobotConfigLoadResult robot =
      roborun::LoadRobotConfiguration(Write("robot.json", ValidRobot()));
  const roborun::PointCatalogLoadResult points =
      roborun::LoadPointCatalog(Write("points.json", ValidPoints()));
  ASSERT_TRUE(robot.configuration.has_value());
  ASSERT_TRUE(points.catalog.has_value());

  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ UNKNOWN SPEED 0\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  const roborun::ValidationResult validation =
      roborun::ValidateProgram(parsed.program, &*robot.configuration, &*points.catalog);
  ASSERT_EQ(validation.diagnostics.size(), 2U);
  EXPECT_EQ(validation.diagnostics[0].code, roborun::DiagnosticCode::kInvalidSpeed);
  EXPECT_EQ(validation.diagnostics[1].code, roborun::DiagnosticCode::kUnknownPoint);
}

TEST(RoboRunValidatorTest, RejectsMoveBeforeServoAndCommandAfterStop) {
  const roborun::ParseResult parsed = Parse("MOVEJ 0 0 0 0 0 0\nSTOP\nMOVEJ 0 0 0 0 0 0\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  const roborun::ValidationResult validation = ValidateProgram(parsed.program);

  ASSERT_EQ(validation.diagnostics.size(), 2U);
  EXPECT_EQ(validation.diagnostics[0].code, roborun::DiagnosticCode::kInvalidRuntimeState);
  EXPECT_EQ(validation.diagnostics[1].line, 3U);
}

TEST(RoboRunRuntimeTest, ServoOffCanBeFollowedByServoOnAndStop) {
  const roborun::ParseResult parsed =
      Parse("SERVO_ON\nSERVO_OFF\nSERVO_ON\nMOVEJ 0 0 0 0 0 0\nSERVO_OFF\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  EXPECT_TRUE(ValidateProgram(parsed.program).diagnostics.empty());

  roborun::MockRobotBackend backend;
  roborun::RobotTaskRuntime runtime;
  const roborun::ExecutionResult result = runtime.Execute(parsed.program, &backend);
  EXPECT_TRUE(result.succeeded);
  EXPECT_EQ(result.final_state, roborun::RuntimeState::kStopped);
}

TEST(RoboRunRuntimeTest, ServoOffOutsideReadyProducesStableDiagnostic) {
  const roborun::ParseResult parsed = Parse("SERVO_OFF\nSTOP\n");
  const roborun::ValidationResult validation = ValidateProgram(parsed.program);
  ASSERT_EQ(validation.diagnostics.size(), 1U);
  EXPECT_EQ(validation.diagnostics[0].code, roborun::DiagnosticCode::kInvalidRuntimeState);
}

TEST(RoboRunRuntimeTest, DelayAdvancesVirtualBusinessTimeWithoutWallClockWaiting) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nDELAY 500\nDELAY 250\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  roborun::MockRobotBackend backend;
  roborun::RobotTaskRuntime runtime;
  const roborun::ExecutionResult result = runtime.Execute(parsed.program, &backend);

  ASSERT_TRUE(result.succeeded);
  EXPECT_EQ(result.finished_business_time_ms, 750);
  ASSERT_EQ(result.trace.size(), 6U);
  EXPECT_EQ(result.trace[1].command_status, roborun::CommandExecutionStatus::kAccepted);
  EXPECT_EQ(result.trace[2].business_time_ms, 500);
  EXPECT_EQ(result.trace[3].command_status, roborun::CommandExecutionStatus::kAccepted);
  EXPECT_EQ(result.trace[4].business_time_ms, 750);
  EXPECT_EQ(result.trace.back().business_time_ms, 750);
}

TEST(RoboRunRuntimeTest, ZeroDelayCompletesAtCurrentBusinessTime) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nDELAY 0\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  roborun::MockRobotBackend backend;
  const roborun::ExecutionResult result =
      roborun::RobotTaskRuntime().Execute(parsed.program, &backend);
  ASSERT_TRUE(result.succeeded);
  EXPECT_EQ(result.finished_business_time_ms, 0);
  EXPECT_EQ(result.trace[1].business_time_ms, 0);
  EXPECT_EQ(result.trace[1].command_status, roborun::CommandExecutionStatus::kSucceeded);
}

TEST(RoboRunClockTest, VirtualClockStartsAtZeroAndRejectsBackwardTime) {
  roborun::VirtualClock clock;
  EXPECT_EQ(clock.Now(), 0);

  clock.AdvanceBy(25);
  EXPECT_EQ(clock.Now(), 25);
  EXPECT_THROW(clock.AdvanceTo(24), std::invalid_argument);
  EXPECT_THROW(clock.AdvanceBy(-1), std::invalid_argument);
}

TEST(RoboRunEventQueueTest, OrdersTimeThenPriorityThenInsertionSequence) {
  roborun::EventQueue events;
  events.Schedule(roborun::EventScheduleRequest::Simple(10, roborun::RuntimeEventPriority::kTimeout,
                                                        roborun::RuntimeEventType::kMotionPoll));
  events.Schedule(
      roborun::EventScheduleRequest::Simple(10, roborun::RuntimeEventPriority::kExternalFeedback,
                                            roborun::RuntimeEventType::kMotionPoll));
  events.Schedule(
      roborun::EventScheduleRequest::Simple(10, roborun::RuntimeEventPriority::kExternalFeedback,
                                            roborun::RuntimeEventType::kMotionPoll));
  events.Schedule(roborun::EventScheduleRequest::Simple(5, roborun::RuntimeEventPriority::kTimeout,
                                                        roborun::RuntimeEventType::kMotionPoll));

  const roborun::RuntimeEvent earliest = *events.PopNext();
  const roborun::RuntimeEvent first = *events.PopNext();
  const roborun::RuntimeEvent second = *events.PopNext();
  const roborun::RuntimeEvent third = *events.PopNext();

  EXPECT_EQ(earliest.time, 5);
  EXPECT_EQ(first.time, 10);
  EXPECT_EQ(first.priority, roborun::RuntimeEventPriority::kExternalFeedback);
  EXPECT_EQ(first.sequence, 1U);
  EXPECT_EQ(second.sequence, 2U);
  EXPECT_EQ(third.priority, roborun::RuntimeEventPriority::kTimeout);
}

class PollingBackend final : public roborun::RobotBackend {
 public:
  std::string Name() const override { return "polling"; }
  std::string Version() const override { return "test"; }
  void Connect() override { connected = true; }
  void SetServoEnabled(bool enabled) override { servo_enabled = enabled; }
  roborun::MotionSubmission SubmitMoveJ(const roborun::MoveJRequest& request) override {
    last_request = request;
    return {roborun::MotionSubmissionStatus::kAccepted, "", std::nullopt};
  }
  roborun::MotionSnapshot PollMotion() override {
    ++polls;
    if (polls == 1) {
      return {roborun::MotionStatus::kRunning, {}, 0, "", 0};
    }
    return {roborun::MotionStatus::kSucceeded, last_request.targets, 2, "", std::nullopt};
  }
  roborun::StopSubmission SubmitStop(roborun::StopReason) override {
    ++stop_requests;
    return {roborun::StopSubmissionStatus::kAccepted, "", 0};
  }
  roborun::StopSnapshot PollStop() override {
    return {roborun::StopStatus::kStopped, {}, 0, "", std::nullopt};
  }
  void Disconnect() override { connected = false; }

  bool connected = false;
  bool servo_enabled = false;
  int polls = 0;
  int stop_requests = 0;
  roborun::MoveJRequest last_request{};
};

TEST(RoboRunRuntimeTest, DoesNotAdvancePastMoveUntilSuccessfulFeedback) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ 0.1 -0.2 0.3 -0.4 0.5 -0.6\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::VirtualClock clock;
  PollingBackend backend;
  roborun::RobotTaskRuntime runtime;
  auto execution = runtime.Start(parsed.program, &backend, &clock);

  EXPECT_EQ(execution.Advance(), roborun::ExecutionAdvanceResult::kProgressed);
  EXPECT_EQ(execution.Advance(), roborun::ExecutionAdvanceResult::kProgressed);
  EXPECT_EQ(execution.backend_state(), roborun::BackendState::kConnected);
  EXPECT_EQ(execution.backend_operation_state(), roborun::BackendOperationState::kActive);
  ASSERT_TRUE(execution.active_command().has_value());
  EXPECT_EQ(execution.active_command()->status, roborun::CommandExecutionStatus::kAccepted);

  clock.AdvanceBy(17);
  EXPECT_EQ(execution.Advance(), roborun::ExecutionAdvanceResult::kProgressed);
  ASSERT_TRUE(execution.active_command().has_value());
  EXPECT_EQ(execution.active_command()->status, roborun::CommandExecutionStatus::kRunning);
  EXPECT_EQ(execution.result().trace.back().command, roborun::CommandType::kMoveJ);
  EXPECT_EQ(execution.result().trace.back().command_status,
            roborun::CommandExecutionStatus::kRunning);
  EXPECT_EQ(execution.result().trace.back().business_time_ms, 17);

  EXPECT_EQ(execution.Advance(), roborun::ExecutionAdvanceResult::kProgressed);
  EXPECT_FALSE(execution.active_command().has_value());
  EXPECT_EQ(execution.result().trace.back().command_status,
            roborun::CommandExecutionStatus::kSucceeded);
  EXPECT_EQ(execution.Advance(), roborun::ExecutionAdvanceResult::kProgressed);
  EXPECT_EQ(execution.Advance(), roborun::ExecutionAdvanceResult::kProgressed);
  EXPECT_EQ(execution.Advance(), roborun::ExecutionAdvanceResult::kTerminal);
  EXPECT_TRUE(execution.result().succeeded);
  EXPECT_EQ(execution.result().final_state, roborun::RuntimeState::kStopped);
  EXPECT_EQ(execution.backend_state(), roborun::BackendState::kDisconnected);
  EXPECT_EQ(execution.backend_operation_state(), roborun::BackendOperationState::kStopped);
}

TEST(RoboRunRuntimeSessionTest, OwnsOneProgramAndKeepsOneShotExecutionCompatible) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nDELAY 5\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::VirtualClock clock;
  roborun::MockRobotBackend backend;
  roborun::RuntimeSession session(&backend, &clock);

  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kIdle);
  EXPECT_EQ(session.SubmitProgram(parsed.program).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  EXPECT_EQ(session.SubmitProgram(parsed.program).status,
            roborun::ProgramSubmissionStatus::kRejected);

  while (!session.is_terminal()) {
    session.Advance();
  }

  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kStopped);
  EXPECT_TRUE(session.result().succeeded);
  EXPECT_EQ(session.result().final_state, roborun::RuntimeState::kStopped);
  EXPECT_EQ(session.result().finished_business_time_ms, 5);

  EXPECT_EQ(session.SubmitProgram(parsed.program).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  while (!session.is_terminal()) {
    session.Advance();
  }
  EXPECT_EQ(session.result().finished_business_time_ms, 10);
}

TEST(RoboRunRuntimeSessionTest, QueuesLifecycleControlsBeforeDeviceFeedback) {
  roborun::EventQueue events;
  events.Schedule(
      roborun::EventScheduleRequest::Simple(10, roborun::RuntimeEventPriority::kExternalFeedback,
                                            roborun::RuntimeEventType::kMotionPoll));
  events.ScheduleControl(10, roborun::ControlType::kPause, 1, "operator");
  events.ScheduleControl(10, roborun::ControlType::kStop, 2, "operator");
  events.ScheduleControl(10, roborun::ControlType::kEstop, 3, "operator");
  events.ScheduleControl(10, roborun::ControlType::kResume, 4, "operator");
  events.ScheduleControl(10, roborun::ControlType::kReset, 5, "operator");

  EXPECT_EQ(events.PopNext()->control_type, roborun::ControlType::kEstop);
  EXPECT_EQ(events.PopNext()->control_type, roborun::ControlType::kStop);
  EXPECT_EQ(events.PopNext()->control_type, roborun::ControlType::kPause);
  EXPECT_EQ(events.PopNext()->control_type, roborun::ControlType::kResume);
  EXPECT_EQ(events.PopNext()->control_type, roborun::ControlType::kReset);
  EXPECT_EQ(events.PopNext()->type, roborun::RuntimeEventType::kMotionPoll);
}

TEST(RoboRunRuntimeSessionTest, PausesBeforeStartingTheNextCommandThroughTheControlQueue) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());

  roborun::VirtualClock clock;
  roborun::MockRobotBackend backend;
  roborun::RuntimeSession session(&backend, &clock);
  ASSERT_EQ(session.SubmitProgram(parsed.program).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  EXPECT_EQ(session.SubmitControl(roborun::ControlType::kPause, "operator").status,
            roborun::ControlSubmissionStatus::kQueued);

  session.Advance();

  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPaused);
  ASSERT_FALSE(session.result().trace.empty());
  EXPECT_EQ(session.result().trace.back().control_type, roborun::ControlType::kPause);
  EXPECT_EQ(session.result().trace.back().control_status,
            roborun::ControlSubmissionStatus::kAccepted);
  EXPECT_EQ(session.backend_state(), roborun::BackendState::kDisconnected);

  EXPECT_EQ(session.SubmitControl(roborun::ControlType::kResume, "operator").status,
            roborun::ControlSubmissionStatus::kQueued);
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kRunning);
  EXPECT_EQ(session.result().trace.back().control_type, roborun::ControlType::kResume);
  EXPECT_EQ(session.result().trace.back().control_status,
            roborun::ControlSubmissionStatus::kAccepted);
}

TEST(RoboRunRuntimeSessionTest, RecordsDeferredStopEmergencyAndResetRequests) {
  roborun::VirtualClock clock;
  roborun::MockRobotBackend backend;
  roborun::RuntimeSession session(&backend, &clock);

  session.SubmitControl(roborun::ControlType::kStop, "operator");
  session.SubmitControl(roborun::ControlType::kEstop, "operator");
  session.SubmitControl(roborun::ControlType::kReset, "operator");
  session.Advance();
  session.Advance();
  session.Advance();

  ASSERT_EQ(session.result().trace.size(), 3U);
  EXPECT_EQ(session.result().trace[0].control_type, roborun::ControlType::kEstop);
  EXPECT_EQ(session.result().trace[0].control_status, roborun::ControlSubmissionStatus::kAccepted);
  EXPECT_EQ(session.result().trace[1].control_type, roborun::ControlType::kStop);
  EXPECT_EQ(session.result().trace[1].control_status, roborun::ControlSubmissionStatus::kIgnored);
  EXPECT_EQ(session.result().trace[2].control_type, roborun::ControlType::kReset);
  EXPECT_EQ(session.result().trace[2].control_status, roborun::ControlSubmissionStatus::kAccepted);
  EXPECT_EQ(session.backend_state(), roborun::BackendState::kDisconnected);
}

TEST(RoboRunRuntimeSessionTest, RejectedProgramDoesNotBlockTheNextSubmission) {
  const roborun::ParseResult invalid = Parse("MOVEJ 0 0 0 0 0 0\nSTOP\n");
  const roborun::ParseResult valid = Parse("SERVO_ON\nSTOP\n");
  ASSERT_TRUE(invalid.diagnostics.empty());
  ASSERT_TRUE(valid.diagnostics.empty());
  roborun::VirtualClock clock;
  roborun::MockRobotBackend backend;
  roborun::RuntimeSession session(&backend, &clock);

  EXPECT_EQ(session.SubmitProgram(invalid.program).status,
            roborun::ProgramSubmissionStatus::kRejected);
  EXPECT_TRUE(session.is_terminal());
  EXPECT_EQ(session.SubmitProgram(valid.program).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  while (!session.is_terminal()) {
    session.Advance();
  }

  EXPECT_TRUE(session.result().succeeded);
}

TEST(RoboRunRuntimeTest, ZeroDelayBackendPollingCannotStarveTimeouts) {
  const roborun::ParseResult parsed = Parse("SERVO_ON\nMOVEJ 0 0 0 0 0 0 TIMEOUT 3\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  ZeroDelayPollingBackend backend;
  roborun::VirtualClock clock;
  roborun::RuntimeOptions options;
  options.stop_confirmation_timeout_ms = 2;
  roborun::RuntimeExecution execution(&backend, &clock, nullptr, nullptr, nullptr, nullptr, nullptr,
                                      options);
  ASSERT_EQ(execution.SubmitProgram(parsed.program).status,
            roborun::ProgramSubmissionStatus::kAccepted);

  for (int step = 0; !execution.is_terminal() && step < 20; ++step) {
    execution.Advance();
  }

  EXPECT_TRUE(execution.is_terminal());
  ASSERT_TRUE(execution.result().primary_alarm_index.has_value());
  EXPECT_EQ(execution.result().alarms[*execution.result().primary_alarm_index].code,
            roborun::DiagnosticCode::kMotionTimeout);
}

TEST(RoboRunRuntimeTest, BusinessTimeOverflowBecomesAStableDiagnostic) {
  const std::string max_time = std::to_string(std::numeric_limits<roborun::BusinessTime>::max());
  const roborun::ParseResult parsed = Parse("SERVO_ON\nDELAY " + max_time + "\nDELAY 1\nSTOP\n");
  ASSERT_TRUE(parsed.diagnostics.empty());
  roborun::MockRobotBackend backend;
  const roborun::RobotTaskRuntime runtime;

  const roborun::ExecutionResult result = runtime.Execute(parsed.program, &backend);

  ASSERT_FALSE(result.diagnostics.empty());
  EXPECT_EQ(result.diagnostics.front().code, roborun::DiagnosticCode::kInvalidDuration);
}

TEST(RoboRunRuntimeSessionTest, ControlOnlyTraceHasNoFabricatedCommand) {
  roborun::VirtualClock clock;
  roborun::MockRobotBackend backend;
  roborun::RuntimeSession session(&backend, &clock);

  session.SubmitControl(roborun::ControlType::kEstop, "operator");
  session.Advance();

  ASSERT_FALSE(session.result().trace.empty());
  EXPECT_EQ(roborun::ToString(session.result().trace.front().command), "NONE");
  EXPECT_EQ(session.result().trace.front().line, 0U);
}

}  // namespace
