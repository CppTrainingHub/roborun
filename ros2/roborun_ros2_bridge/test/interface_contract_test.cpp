#include <gtest/gtest.h>

#include "roborun_interfaces/action/execute_program.hpp"
#include "roborun_interfaces/msg/alarm_state.hpp"
#include "roborun_interfaces/msg/io_state.hpp"
#include "roborun_interfaces/msg/runtime_status.hpp"
#include "roborun_interfaces/srv/control_runtime.hpp"

TEST(RoboRunRos2InterfaceContractTest, MessagesKeepSnapshotAndBusinessTimeSeparate) {
  roborun_interfaces::action::ExecuteProgram::Feedback feedback;
  roborun_interfaces::msg::RuntimeStatus status;
  roborun_interfaces::msg::IoState io;
  roborun_interfaces::msg::AlarmState alarms;

  feedback.snapshot_sequence = 42;
  feedback.business_time_ms = 1200;
  status.snapshot_sequence = feedback.snapshot_sequence;
  status.business_time_ms = feedback.business_time_ms;
  io.snapshot_sequence = feedback.snapshot_sequence;
  io.business_time_ms = feedback.business_time_ms;
  alarms.snapshot_sequence = feedback.snapshot_sequence;
  alarms.business_time_ms = feedback.business_time_ms;

  EXPECT_EQ(status.snapshot_sequence, 42U);
  EXPECT_EQ(status.business_time_ms, 1200);
  EXPECT_EQ(io.snapshot_sequence, status.snapshot_sequence);
  EXPECT_EQ(alarms.business_time_ms, status.business_time_ms);
}

TEST(RoboRunRos2InterfaceContractTest, ControlResponseCarriesTheRuntimeRequestIdentity) {
  roborun_interfaces::srv::ControlRuntime::Response response;
  response.request_id = 7;
  response.control = "ESTOP";
  response.status = "queued";
  response.message = "";

  EXPECT_EQ(response.request_id, 7U);
  EXPECT_EQ(response.control, "ESTOP");
  EXPECT_EQ(response.status, "queued");
}
