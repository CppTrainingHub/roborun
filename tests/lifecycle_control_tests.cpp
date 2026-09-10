#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "roborun/clock.h"
#include "roborun/parser.h"
#include "roborun/runtime.h"
#include "roborun/scenario.h"
#include "roborun/scripted_mock_backend.h"

namespace {

roborun::Program ParseProgram(const std::string& text) {
  std::istringstream input(text);
  const roborun::ParseResult parsed = roborun::ParseProgram(input, "lifecycle-control.task");
  EXPECT_TRUE(parsed.diagnostics.empty());
  return parsed.program;
}

void AdvanceToTerminal(roborun::RuntimeSession* session) {
  for (int step = 0; step < 100; ++step) {
    if (session->is_terminal()) {
      return;
    }
    session->Advance();
  }
  FAIL() << "Runtime Session did not reach a terminal task state";
}

class CleanupFailingBackend final : public roborun::RobotBackend {
 public:
  std::string Name() const override { return "cleanup-failing"; }
  std::string Version() const override { return "test"; }
  void Connect() override { connected_ = true; }
  void SetServoEnabled(bool enabled) override {
    if (!enabled) {
      throw std::runtime_error("servo disable failed");
    }
  }
  roborun::MotionSubmission SubmitMoveJ(const roborun::MoveJRequest&) override {
    return {roborun::MotionSubmissionStatus::kRejected, "motion rejected", std::nullopt};
  }
  roborun::MotionSnapshot PollMotion() override {
    return {roborun::MotionStatus::kFailed, {}, 0, "unexpected motion poll", std::nullopt};
  }
  roborun::StopSubmission SubmitStop(roborun::StopReason) override {
    return {roborun::StopSubmissionStatus::kAccepted, "", 0};
  }
  roborun::StopSnapshot PollStop() override {
    return {roborun::StopStatus::kStopped, {}, 0, "", std::nullopt};
  }
  void Disconnect() override {
    if (connected_) {
      throw std::runtime_error("disconnect failed");
    }
  }

 private:
  bool connected_ = false;
};

TEST(LifecycleControlTest, UnrelatedFeedbackCannotCompleteAnotherNamedWait) {
  for (bool tool_wait : {false, true}) {
    roborun::IOConfiguration io;
    roborun::ToolConfiguration tools;
    for (const std::string name : {"TARGET", "OTHER"}) {
      io.signals.emplace(name,
                         roborun::DigitalSignal{name, roborun::SignalDirection::kDi, false, 0});
      tools.tools.emplace(
          name, roborun::ToolState{name, roborun::ToolType::kGripper, roborun::ToolPosition::kOpen,
                                   roborun::ToolPosition::kOpen, 0});
    }
    roborun::MockScenario scenario;
    for (const auto& [time, name] : {std::pair{10, "OTHER"}, std::pair{20, "TARGET"}}) {
      roborun::ScenarioEvent event;
      event.time_ms = time;
      event.kind = tool_wait ? roborun::ScenarioEventKind::kToolFeedback
                             : roborun::ScenarioEventKind::kDiChange;
      event.name = name;
      event.value = true;
      event.tool_position = roborun::ToolPosition::kClosed;
      scenario.events.push_back(event);
    }
    roborun::ScriptedMockBackend backend(scenario);
    roborun::VirtualClock clock;
    roborun::RuntimeSession session(&backend, &clock, nullptr, nullptr, &io, &tools, &scenario);
    const std::string wait = tool_wait ? "SET_TOOL TARGET CLOSED" : "WAIT_DI TARGET ON";
    ASSERT_EQ(
        session.SubmitProgram(ParseProgram("SERVO_ON\n" + wait + " TIMEOUT 30\nSTOP\n")).status,
        roborun::ProgramSubmissionStatus::kAccepted);
    AdvanceToTerminal(&session);
    EXPECT_TRUE(session.result().succeeded);
    EXPECT_EQ(session.result().finished_business_time_ms, 20);
  }
}

TEST(LifecycleControlTest, LocalCompletionCannotEscapeStopOrEstop) {
  for (auto control : {roborun::ControlType::kStop, roborun::ControlType::kEstop}) {
    for (const std::string wait :
         {"DELAY 10", "WAIT_DI READY ON TIMEOUT 30", "SET_TOOL GRIPPER CLOSED TIMEOUT 30"}) {
      SCOPED_TRACE(wait + roborun::ToString(control));
      roborun::IOConfiguration io;
      io.signals.emplace("READY",
                         roborun::DigitalSignal{"READY", roborun::SignalDirection::kDi, false, 0});
      roborun::ToolConfiguration tools;
      tools.tools.emplace("GRIPPER", roborun::ToolState{"GRIPPER", roborun::ToolType::kGripper,
                                                        roborun::ToolPosition::kOpen,
                                                        roborun::ToolPosition::kOpen, 0});
      roborun::MockScenario scenario;
      scenario.schema_version = 2;
      scenario.events.push_back({10, roborun::ScenarioEventKind::kDiChange, "READY", true});
      roborun::ScenarioEvent feedback;
      feedback.time_ms = 10;
      feedback.kind = roborun::ScenarioEventKind::kToolFeedback;
      feedback.name = "GRIPPER";
      feedback.tool_position = roborun::ToolPosition::kClosed;
      scenario.events.push_back(feedback);
      scenario.stop_plans.emplace(
          1, roborun::StopPlan{1, roborun::StopPlanOutcome::kDelayedConfirmation, 20});
      roborun::ScriptedMockBackend backend(scenario);
      roborun::VirtualClock clock;
      roborun::RuntimeSession session(&backend, &clock, nullptr, nullptr, &io, &tools, &scenario);
      ASSERT_EQ(
          session.SubmitProgram(ParseProgram("SERVO_ON\n" + wait + "\nSERVO_OFF\nSTOP\n")).status,
          roborun::ProgramSubmissionStatus::kAccepted);
      session.Advance();
      session.Advance();
      session.SubmitControlAt(1, control, "test");
      for (int step = 0; step < 50 && !session.is_terminal(); ++step) {
        session.Advance();
        if (!session.is_terminal()) {
          EXPECT_EQ(session.lifecycle_state(), control == roborun::ControlType::kStop
                                                   ? roborun::LifecycleState::kStopping
                                                   : roborun::LifecycleState::kEmergencyStopping);
          ASSERT_TRUE(session.active_command().has_value());
          EXPECT_EQ(session.active_command()->command_index, 1U);
        }
      }
      ASSERT_TRUE(session.is_terminal());
      EXPECT_EQ(session.result().finished_business_time_ms, 21);
      for (const auto& entry : session.result().trace) {
        if (entry.command_index == 2) {
          EXPECT_NE(entry.command_status, roborun::CommandExecutionStatus::kSucceeded);
        }
      }
    }
  }
}

TEST(LifecycleControlTest, ResumeKeepsMotionPlanAndNextProgramRestartsPlans) {
  roborun::MockScenario scenario;
  scenario.motion_plans.emplace(
      1, roborun::MotionPlan{1, roborun::MotionPlanOutcome::kDelayedSuccess, 10});
  scenario.motion_plans.emplace(2,
                                roborun::MotionPlan{2, roborun::MotionPlanOutcome::kRejected, 0});
  roborun::ScriptedMockBackend backend(scenario);
  roborun::VirtualClock clock;
  roborun::RuntimeOptions options;
  options.disconnect_on_terminal = false;
  roborun::RuntimeSession session(&backend, &clock, nullptr, nullptr, nullptr, nullptr, nullptr,
                                  options);
  const auto program = ParseProgram("SERVO_ON\nMOVEJ 0.2 0 0 0 0 0\nMOVEJ 0.4 0 0 0 0 0\nSTOP\n");
  for (int attempt = 0; attempt < 2; ++attempt) {
    ASSERT_EQ(session.SubmitProgram(program).status, roborun::ProgramSubmissionStatus::kAccepted);
    session.Advance();
    session.Advance();
    session.SubmitControl(roborun::ControlType::kPause, "test");
    for (int step = 0; step < 10 && session.lifecycle_state() != roborun::LifecycleState::kPaused;
         ++step) {
      session.Advance();
    }
    ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPaused);
    session.SubmitControl(roborun::ControlType::kResume, "test");
    AdvanceToTerminal(&session);
    EXPECT_EQ(backend.positions()[0], 0.2);
    ASSERT_TRUE(session.result().primary_alarm_index.has_value());
    EXPECT_EQ(session.result().alarms[*session.result().primary_alarm_index].code,
              roborun::DiagnosticCode::kMotionRejected);
    const auto success = std::find_if(
        session.result().trace.begin(), session.result().trace.end(), [](const auto& entry) {
          return entry.command_index == 2 &&
                 entry.command_status == roborun::CommandExecutionStatus::kSucceeded;
        });
    EXPECT_NE(success, session.result().trace.end());
    EXPECT_EQ(backend.stop_requests(), 2);
    session.SubmitControl(roborun::ControlType::kReset, "test");
    session.Advance();
    ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kIdle);
  }
}

TEST(LifecycleControlTest, ExternalStopWaitsForConfirmationAndReportsOperatorOutcome) {
  roborun::MockScenario scenario;
  scenario.stop_plans.emplace(
      1, roborun::StopPlan{1, roborun::StopPlanOutcome::kDelayedConfirmation, 15});
  scenario.motion_plans.emplace(1, roborun::MotionPlan{1, roborun::MotionPlanOutcome::kFrozen, 0});
  roborun::ScriptedMockBackend backend(scenario);
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);

  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n")).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  session.Advance();
  session.Advance();
  ASSERT_TRUE(session.active_command().has_value());

  ASSERT_EQ(session.SubmitControl(roborun::ControlType::kStop, "operator").status,
            roborun::ControlSubmissionStatus::kQueued);
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kStopping);
  EXPECT_EQ(backend.stop_requests(), 1);
  EXPECT_FALSE(session.is_terminal());
  ASSERT_FALSE(session.result().trace.empty());
  EXPECT_EQ(session.result().trace.back().lifecycle_before, roborun::LifecycleState::kRunning);
  EXPECT_EQ(session.result().trace.back().lifecycle_after, roborun::LifecycleState::kStopping);

  AdvanceToTerminal(&session);
  EXPECT_EQ(session.result().outcome, roborun::ExecutionOutcome::kOperatorStopped);
  EXPECT_FALSE(session.result().succeeded);
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kStopped);
  EXPECT_EQ(session.result().finished_business_time_ms, 15);
}

TEST(LifecycleControlTest, PauseDelayPreservesRemainingBusinessTime) {
  roborun::ScriptedMockBackend backend(roborun::MockScenario{});
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nDELAY 100\nSTOP\n")).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  session.Advance();
  session.Advance();
  clock.AdvanceTo(40);

  session.SubmitControl(roborun::ControlType::kPause, "operator");
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPaused);
  ASSERT_TRUE(session.active_command().has_value());
  EXPECT_EQ(session.active_command()->status, roborun::CommandExecutionStatus::kPaused);

  clock.AdvanceTo(70);
  session.SubmitControl(roborun::ControlType::kResume, "operator");
  session.Advance();
  AdvanceToTerminal(&session);
  EXPECT_TRUE(session.result().succeeded);
  EXPECT_EQ(session.result().finished_business_time_ms, 130);
}

TEST(LifecycleControlTest, EstopDominatesStopAndLatchesAnEmergencyAlarm) {
  roborun::MockScenario scenario;
  scenario.motion_plans.emplace(1, roborun::MotionPlan{1, roborun::MotionPlanOutcome::kFrozen, 0});
  roborun::ScriptedMockBackend backend(scenario);
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n")).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  session.Advance();
  session.Advance();
  session.SubmitControl(roborun::ControlType::kStop, "operator");
  session.SubmitControl(roborun::ControlType::kEstop, "safety");

  AdvanceToTerminal(&session);
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kEmergencyStopped);
  EXPECT_EQ(session.result().outcome, roborun::ExecutionOutcome::kEmergencyStopped);
  ASSERT_FALSE(session.result().alarms.empty());
  EXPECT_EQ(session.result().alarms.front().severity, roborun::AlarmSeverity::kEmergency);
  EXPECT_EQ(backend.stop_requests(), 1);
}

TEST(LifecycleControlTest, ResetClearsTheLatchAndAllowsASecondProgram) {
  roborun::ScriptedMockBackend backend(roborun::MockScenario{});
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  const roborun::Program program = ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n");
  ASSERT_EQ(session.SubmitProgram(program).status, roborun::ProgramSubmissionStatus::kAccepted);
  session.Advance();
  session.Advance();
  session.SubmitControl(roborun::ControlType::kEstop, "safety");
  AdvanceToTerminal(&session);

  session.SubmitControl(roborun::ControlType::kReset, "operator");
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kIdle);
  ASSERT_FALSE(session.result().alarms.empty());
  EXPECT_FALSE(session.result().alarms.front().active);

  ASSERT_EQ(session.SubmitProgram(program).status, roborun::ProgramSubmissionStatus::kAccepted);
  AdvanceToTerminal(&session);
  EXPECT_TRUE(session.result().succeeded);
  EXPECT_EQ(session.result().outcome, roborun::ExecutionOutcome::kProgramCompleted);
}

TEST(LifecycleControlTest, MovePauseResubmitsTheSameTargetAfterConfirmedStop) {
  roborun::MockScenario scenario;
  scenario.motion_plans.emplace(
      1, roborun::MotionPlan{1, roborun::MotionPlanOutcome::kDelayedSuccess, 10});
  roborun::ScriptedMockBackend backend(scenario);
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0.2 0 0 0 0 0 TIMEOUT 100\nSTOP\n"))
                .status,
            roborun::ProgramSubmissionStatus::kAccepted);
  session.Advance();
  session.Advance();
  session.SubmitControl(roborun::ControlType::kPause, "operator");
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPausing);
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPaused);

  session.SubmitControl(roborun::ControlType::kResume, "operator");
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kRunning);
  AdvanceToTerminal(&session);
  EXPECT_TRUE(session.result().succeeded);
  int submit_count = 0;
  for (const std::string& call : backend.call_log()) {
    if (call == "submit_movej") {
      ++submit_count;
    }
  }
  EXPECT_EQ(submit_count, 2);
}

TEST(LifecycleControlTest, PausedWaitRecordsFeedbackAndCompletesImmediatelyAfterResume) {
  roborun::IOConfiguration io;
  io.signals.emplace("PART_READY",
                     roborun::DigitalSignal{"PART_READY", roborun::SignalDirection::kDi, false, 0});
  roborun::MockScenario scenario;
  scenario.events.push_back(
      roborun::ScenarioEvent{10, roborun::ScenarioEventKind::kDiChange, "PART_READY", true});
  roborun::ScriptedMockBackend backend(scenario);
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock, nullptr, nullptr, &io, nullptr, &scenario);
  ASSERT_EQ(
      session.SubmitProgram(ParseProgram("SERVO_ON\nWAIT_DI PART_READY ON TIMEOUT 100\nSTOP\n"))
          .status,
      roborun::ProgramSubmissionStatus::kAccepted);
  session.Advance();
  session.Advance();
  session.SubmitControl(roborun::ControlType::kPause, "operator");
  session.Advance();
  ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPaused);
  session.SubmitControlAt(25, roborun::ControlType::kResume, "operator");
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPaused);
  session.Advance();
  EXPECT_FALSE(session.active_command().has_value());
  AdvanceToTerminal(&session);
  EXPECT_TRUE(session.result().succeeded);
}

TEST(LifecycleControlTest, StopRejectionAndConfirmationTimeoutHaveStableAlarmCodes) {
  const roborun::Program program = ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n");
  roborun::MockScenario rejected;
  rejected.motion_plans.emplace(1, roborun::MotionPlan{1, roborun::MotionPlanOutcome::kFrozen, 0});
  rejected.stop_plans.emplace(1, roborun::StopPlan{1, roborun::StopPlanOutcome::kRejected, 0});
  roborun::ScriptedMockBackend rejected_backend(rejected);
  roborun::VirtualClock rejected_clock;
  roborun::RuntimeSession rejected_session(&rejected_backend, &rejected_clock);
  ASSERT_EQ(rejected_session.SubmitProgram(program).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  rejected_session.Advance();
  rejected_session.Advance();
  rejected_session.SubmitControl(roborun::ControlType::kStop, "operator");
  AdvanceToTerminal(&rejected_session);
  ASSERT_TRUE(rejected_session.result().primary_alarm_index.has_value());
  EXPECT_EQ(rejected_session.result().alarms[*rejected_session.result().primary_alarm_index].code,
            roborun::DiagnosticCode::kStopRejected);

  roborun::MockScenario frozen;
  frozen.motion_plans.emplace(1, roborun::MotionPlan{1, roborun::MotionPlanOutcome::kFrozen, 0});
  frozen.stop_plans.emplace(1, roborun::StopPlan{1, roborun::StopPlanOutcome::kNeverConfirmed, 0});
  roborun::ScriptedMockBackend frozen_backend(frozen);
  roborun::VirtualClock frozen_clock;
  roborun::RuntimeOptions options;
  options.stop_confirmation_timeout_ms = 20;
  options.default_stop_poll_ms = 5;
  roborun::RuntimeSession frozen_session(&frozen_backend, &frozen_clock, nullptr, nullptr, nullptr,
                                         nullptr, nullptr, options);
  ASSERT_EQ(frozen_session.SubmitProgram(program).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  frozen_session.Advance();
  frozen_session.Advance();
  frozen_session.SubmitControl(roborun::ControlType::kStop, "operator");
  AdvanceToTerminal(&frozen_session);
  ASSERT_TRUE(frozen_session.result().primary_alarm_index.has_value());
  EXPECT_EQ(frozen_session.result().alarms[*frozen_session.result().primary_alarm_index].code,
            roborun::DiagnosticCode::kStopConfirmationTimeout);
}

TEST(LifecycleControlTest, ResetRejectsEveryFaultWhoseStopWasNotConfirmed) {
  roborun::MockScenario scenario;
  scenario.motion_plans.emplace(1, roborun::MotionPlan{1, roborun::MotionPlanOutcome::kFrozen, 0});
  scenario.stop_plans.emplace(1, roborun::StopPlan{1, roborun::StopPlanOutcome::kRejected, 0});
  roborun::ScriptedMockBackend backend(scenario);
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n")).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  session.Advance();
  session.Advance();
  session.SubmitControl(roborun::ControlType::kStop, "operator");
  AdvanceToTerminal(&session);

  ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kFaulted);
  ASSERT_EQ(session.SubmitControl(roborun::ControlType::kReset, "operator").status,
            roborun::ControlSubmissionStatus::kQueued);
  session.Advance();
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kFaulted);
  ASSERT_FALSE(session.result().trace.empty());
  EXPECT_EQ(session.result().trace.back().control_status,
            roborun::ControlSubmissionStatus::kRejected);
}

TEST(LifecycleControlTest, PrimaryAlarmSurvivesServoAndDisconnectCleanupFailures) {
  CleanupFailingBackend backend;
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n")).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  AdvanceToTerminal(&session);

  ASSERT_TRUE(session.result().primary_alarm_index.has_value());
  ASSERT_GE(session.result().alarms.size(), 3U);
  EXPECT_EQ(session.result().alarms[*session.result().primary_alarm_index].code,
            roborun::DiagnosticCode::kMotionRejected);
  EXPECT_EQ(session.result().alarms[1].code, roborun::DiagnosticCode::kServoDisableFailure);
  EXPECT_EQ(session.result().alarms[2].code, roborun::DiagnosticCode::kDisconnectFailure);
  EXPECT_EQ(session.lifecycle_state(), roborun::LifecycleState::kFaulted);
}

TEST(LifecycleScenarioTest, LoadsVersionTwoControlsAndRejectsControlsInVersionOne) {
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() / "roborun-lifecycle-control-scenario";
  std::filesystem::create_directories(directory);
  const std::filesystem::path version_two = directory / "v2.json";
  std::ofstream(version_two) << R"({"schema_version":2,"controls":[
    {"time_ms":5,"action":"PAUSE","source":"operator"},
    {"time_ms":10,"action":"RESUME","source":"operator"}],
    "stops":[{"episode":1,"outcome":"delayed_confirmation","delay_ms":5}]})";
  const roborun::ScenarioLoadResult loaded = roborun::LoadMockScenario(version_two.string());
  ASSERT_TRUE(loaded.diagnostics.empty());
  ASSERT_TRUE(loaded.scenario.has_value());
  EXPECT_EQ(loaded.scenario->controls.size(), 2U);
  EXPECT_EQ(loaded.scenario->stop_plans.size(), 1U);

  const std::filesystem::path version_one = directory / "v1-invalid.json";
  std::ofstream(version_one)
      << R"({"schema_version":1,"controls":[{"time_ms":0,"action":"STOP"}]})";
  const roborun::ScenarioLoadResult invalid = roborun::LoadMockScenario(version_one.string());
  EXPECT_FALSE(invalid.diagnostics.empty());
  std::filesystem::remove_all(directory);
}

enum class MatrixSetup {
  kIdle,
  kRunning,
  kWaiting,
  kPaused,
  kPausing,
  kStopping,
  kStopped,
  kEmergencyStopping,
  kEmergencyStopped,
  kFaulted,
};

struct ControlMatrixCase {
  MatrixSetup setup;
  roborun::ControlType control;
  roborun::ControlSubmissionStatus expected_status;
  roborun::LifecycleState expected_lifecycle;
};

class LifecycleControlMatrixTest : public ::testing::TestWithParam<ControlMatrixCase> {};

TEST_P(LifecycleControlMatrixTest, CoversLegalIdempotentAndRejectedControlTransitions) {
  roborun::MockScenario scenario;
  if (GetParam().setup == MatrixSetup::kFaulted) {
    scenario.motion_plans.emplace(1,
                                  roborun::MotionPlan{1, roborun::MotionPlanOutcome::kRejected, 0});
  } else if (GetParam().setup == MatrixSetup::kRunning ||
             GetParam().setup == MatrixSetup::kPausing ||
             GetParam().setup == MatrixSetup::kStopping ||
             GetParam().setup == MatrixSetup::kEmergencyStopping) {
    scenario.motion_plans.emplace(1,
                                  roborun::MotionPlan{1, roborun::MotionPlanOutcome::kFrozen, 0});
  }
  roborun::ScriptedMockBackend backend(scenario);
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  const auto start_running = [&session] {
    ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nDELAY 100\nSTOP\n")).status,
              roborun::ProgramSubmissionStatus::kAccepted);
    session.Advance();
    session.Advance();
    ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kWaiting);
  };
  const auto start_moving = [&session] {
    ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n")).status,
              roborun::ProgramSubmissionStatus::kAccepted);
    session.Advance();
    session.Advance();
    ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kRunning);
  };

  switch (GetParam().setup) {
    case MatrixSetup::kIdle:
      break;
    case MatrixSetup::kRunning:
      start_moving();
      break;
    case MatrixSetup::kWaiting:
      start_running();
      break;
    case MatrixSetup::kPaused:
      start_running();
      ASSERT_EQ(session.SubmitControl(roborun::ControlType::kPause, "matrix").status,
                roborun::ControlSubmissionStatus::kQueued);
      session.Advance();
      ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPaused);
      break;
    case MatrixSetup::kPausing:
      start_moving();
      session.SubmitControl(roborun::ControlType::kPause, "matrix");
      session.Advance();
      ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kPausing);
      break;
    case MatrixSetup::kStopping:
      start_moving();
      session.SubmitControl(roborun::ControlType::kStop, "matrix");
      session.Advance();
      ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kStopping);
      break;
    case MatrixSetup::kStopped:
      ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nSTOP\n")).status,
                roborun::ProgramSubmissionStatus::kAccepted);
      AdvanceToTerminal(&session);
      ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kStopped);
      break;
    case MatrixSetup::kEmergencyStopping:
      start_moving();
      session.SubmitControl(roborun::ControlType::kEstop, "matrix");
      session.Advance();
      ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kEmergencyStopping);
      break;
    case MatrixSetup::kEmergencyStopped:
      start_running();
      ASSERT_EQ(session.SubmitControl(roborun::ControlType::kEstop, "matrix").status,
                roborun::ControlSubmissionStatus::kQueued);
      AdvanceToTerminal(&session);
      ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kEmergencyStopped);
      break;
    case MatrixSetup::kFaulted:
      ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n")).status,
                roborun::ProgramSubmissionStatus::kAccepted);
      AdvanceToTerminal(&session);
      ASSERT_EQ(session.lifecycle_state(), roborun::LifecycleState::kFaulted);
      break;
  }

  const roborun::ControlSubmission submission = session.SubmitControl(GetParam().control, "matrix");
  ASSERT_EQ(submission.status, roborun::ControlSubmissionStatus::kQueued);
  session.Advance();

  const auto trace = std::find_if(session.result().trace.rbegin(), session.result().trace.rend(),
                                  [&submission](const roborun::TraceEntry& entry) {
                                    return entry.control_request_id == submission.request_id;
                                  });
  ASSERT_NE(trace, session.result().trace.rend());
  EXPECT_EQ(trace->control_status, GetParam().expected_status);
  EXPECT_EQ(session.lifecycle_state(), GetParam().expected_lifecycle);
}

INSTANTIATE_TEST_SUITE_P(
    LifecycleAndControl, LifecycleControlMatrixTest,
    ::testing::
        Values(
            ControlMatrixCase{MatrixSetup::kIdle, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kIdle},
            ControlMatrixCase{MatrixSetup::kIdle, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kIdle},
            ControlMatrixCase{MatrixSetup::kIdle, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kIdle},
            ControlMatrixCase{MatrixSetup::kIdle, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kEmergencyStopped},
            ControlMatrixCase{MatrixSetup::kIdle, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kIdle},
            ControlMatrixCase{MatrixSetup::kRunning, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kPausing},
            ControlMatrixCase{MatrixSetup::kRunning, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kRunning},
            ControlMatrixCase{MatrixSetup::kRunning, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kStopping},
            ControlMatrixCase{MatrixSetup::kRunning, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kRunning, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kRunning},
            ControlMatrixCase{MatrixSetup::kWaiting, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kPaused},
            ControlMatrixCase{MatrixSetup::kWaiting, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kWaiting},
            ControlMatrixCase{MatrixSetup::kWaiting, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kStopping},
            ControlMatrixCase{MatrixSetup::kWaiting, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kWaiting, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kWaiting},
            ControlMatrixCase{MatrixSetup::kPaused, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kPaused},
            ControlMatrixCase{MatrixSetup::kPaused, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kWaiting},
            ControlMatrixCase{MatrixSetup::kPaused, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kStopping},
            ControlMatrixCase{MatrixSetup::kPaused, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kPaused, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kPaused},
            ControlMatrixCase{MatrixSetup::kPausing, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kPausing},
            ControlMatrixCase{MatrixSetup::kPausing, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kPausing},
            ControlMatrixCase{MatrixSetup::kPausing, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kPausing},
            ControlMatrixCase{MatrixSetup::kPausing, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kPausing, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kPausing},
            ControlMatrixCase{MatrixSetup::kStopping, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kStopping},
            ControlMatrixCase{MatrixSetup::kStopping, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kStopping},
            ControlMatrixCase{MatrixSetup::kStopping, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kStopping},
            ControlMatrixCase{MatrixSetup::kStopping, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kStopping, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kStopping},
            ControlMatrixCase{MatrixSetup::kStopped, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kStopped},
            ControlMatrixCase{MatrixSetup::kStopped, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kStopped},
            ControlMatrixCase{MatrixSetup::kStopped, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kStopped},
            ControlMatrixCase{MatrixSetup::kStopped, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kEmergencyStopped},
            ControlMatrixCase{MatrixSetup::kStopped, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kStopped},
            ControlMatrixCase{MatrixSetup::kEmergencyStopping, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kEmergencyStopping, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kEmergencyStopping, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kEmergencyStopping, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kEmergencyStopping, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kEmergencyStopping},
            ControlMatrixCase{MatrixSetup::kEmergencyStopped, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kEmergencyStopped},
            ControlMatrixCase{MatrixSetup::kEmergencyStopped, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kEmergencyStopped},
            ControlMatrixCase{MatrixSetup::kEmergencyStopped, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kEmergencyStopped},
            ControlMatrixCase{MatrixSetup::kEmergencyStopped, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kEmergencyStopped},
            ControlMatrixCase{MatrixSetup::kEmergencyStopped, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kIdle},
            ControlMatrixCase{MatrixSetup::kFaulted, roborun::ControlType::kPause,
                              roborun::ControlSubmissionStatus::kRejected,
                              roborun::LifecycleState::kFaulted},
            ControlMatrixCase{MatrixSetup::kFaulted, roborun::ControlType::kResume,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kFaulted},
            ControlMatrixCase{MatrixSetup::kFaulted, roborun::ControlType::kStop,
                              roborun::ControlSubmissionStatus::kIgnored,
                              roborun::LifecycleState::kFaulted},
            ControlMatrixCase{MatrixSetup::kFaulted, roborun::ControlType::kEstop,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kEmergencyStopped},
            ControlMatrixCase{MatrixSetup::kFaulted, roborun::ControlType::kReset,
                              roborun::ControlSubmissionStatus::kAccepted,
                              roborun::LifecycleState::kIdle}));

}  // namespace
