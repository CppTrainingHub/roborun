#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "roborun/clock.h"
#include "roborun/parser.h"
#include "roborun/runtime.h"
#include "roborun/scripted_mock_backend.h"

namespace {

roborun::Program ParseProgram(const std::string& text) {
  std::istringstream input(text);
  const roborun::ParseResult parsed = roborun::ParseProgram(input, "runtime-snapshot.task");
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

TEST(RuntimeSnapshotTest, IsAnIndependentValueAndDoesNotAdvanceTheRuntime) {
  roborun::ScriptedMockBackend backend(roborun::MockScenario{});
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);

  const roborun::RuntimeSnapshot initial = session.Snapshot();
  EXPECT_EQ(initial.sequence, 0U);
  EXPECT_EQ(initial.lifecycle_state, roborun::LifecycleState::kIdle);
  EXPECT_EQ(clock.Now(), 0);
  EXPECT_EQ(backend.call_log().size(), 0U);

  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n")).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  const roborun::RuntimeSnapshot accepted = session.Snapshot();
  EXPECT_GT(accepted.sequence, initial.sequence);
  EXPECT_EQ(accepted.lifecycle_state, roborun::LifecycleState::kRunning);

  session.Advance();
  const roborun::RuntimeSnapshot servo_on = session.Snapshot();
  EXPECT_GT(servo_on.sequence, accepted.sequence);
  EXPECT_FALSE(servo_on.active_command.has_value());

  session.Advance();
  const roborun::RuntimeSnapshot running = session.Snapshot();
  EXPECT_GT(running.sequence, servo_on.sequence);
  ASSERT_TRUE(running.active_command.has_value());
  EXPECT_EQ(running.active_command->type, roborun::CommandType::kMoveJ);
  EXPECT_EQ(clock.Now(), 0);
  EXPECT_EQ(backend.call_log().size(), 3U);

  const roborun::RuntimeSnapshot copy = running;
  session.Advance();
  EXPECT_EQ(copy.sequence, running.sequence);
  EXPECT_EQ(copy.active_command->type, roborun::CommandType::kMoveJ);
}

TEST(RuntimeSnapshotTest, TerminalSnapshotAgreesWithResultAndTrace) {
  roborun::ScriptedMockBackend backend(roborun::MockScenario{});
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nMOVEJ 0 0 0 0 0 0\nSTOP\n")).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  AdvanceToTerminal(&session);

  const roborun::RuntimeSnapshot snapshot = session.Snapshot();
  ASSERT_TRUE(snapshot.terminal.has_value());
  EXPECT_EQ(snapshot.lifecycle_state, session.result().lifecycle_state);
  EXPECT_EQ(snapshot.terminal->outcome, session.result().outcome);
  EXPECT_EQ(snapshot.terminal->finished_business_time_ms,
            session.result().finished_business_time_ms);
  EXPECT_EQ(snapshot.joint_positions, session.result().trace.back().actual_positions);
}

TEST(RuntimeSnapshotTest, PreservesBridgeAssignedControlRequestIdentity) {
  roborun::ScriptedMockBackend backend(roborun::MockScenario{});
  roborun::VirtualClock clock;
  roborun::RuntimeSession session(&backend, &clock);
  ASSERT_EQ(session.SubmitProgram(ParseProgram("SERVO_ON\nDELAY 10\nSTOP\n")).status,
            roborun::ProgramSubmissionStatus::kAccepted);
  session.Advance();

  const roborun::ControlSubmission submission =
      session.SubmitControlWithRequestId(roborun::ControlType::kPause, 42, "bridge-service");
  EXPECT_EQ(submission.status, roborun::ControlSubmissionStatus::kQueued);
  EXPECT_EQ(submission.request_id, 42U);
  session.Advance();

  ASSERT_FALSE(session.result().trace.empty());
  EXPECT_EQ(session.result().trace.back().control_request_id, 42U);
  EXPECT_EQ(session.result().trace.back().control_source, "bridge-service");
}

}  // namespace
