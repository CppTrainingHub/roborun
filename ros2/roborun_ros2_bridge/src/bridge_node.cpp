#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roborun/config.h"
#include "roborun/mock_robot_backend.h"
#include "roborun/parser.h"
#include "roborun/runtime.h"
#include "roborun/scenario.h"
#include "roborun/scripted_mock_backend.h"
#include "roborun/validator.h"
#include "roborun/workcell.h"
#include "sensor_msgs/msg/joint_state.hpp"
#ifdef ROBORUN_WITH_COPPELIASIM
#include "roborun/coppeliasim_backend.h"
#endif
#include "roborun_interfaces/action/execute_program.hpp"
#include "roborun_interfaces/msg/alarm_state.hpp"
#include "roborun_interfaces/msg/alarm_state_entry.hpp"
#include "roborun_interfaces/msg/io_state.hpp"
#include "roborun_interfaces/msg/runtime_status.hpp"
#include "roborun_interfaces/msg/signal_state.hpp"
#include "roborun_interfaces/msg/tool_state.hpp"
#include "roborun_interfaces/srv/control_runtime.hpp"

namespace {

using ExecuteProgram = roborun_interfaces::action::ExecuteProgram;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteProgram>;
using ControlRuntime = roborun_interfaces::srv::ControlRuntime;

constexpr std::size_t kRequestQueueCapacity = 32;
constexpr std::size_t kMaxProgramBytes = 64 * 1024;
constexpr std::size_t kMaxSourceNameBytes = 256;
constexpr int kShutdownAdvanceLimit = 10000;

struct StationConfiguration {
  std::optional<roborun::RobotConfiguration> robot;
  std::optional<roborun::PointCatalog> points;
  std::optional<roborun::IOConfiguration> io;
  std::optional<roborun::ToolConfiguration> tools;
  std::optional<roborun::MockScenario> scenario;
  std::optional<roborun::WorkcellConfiguration> workcell;
};

struct BackendBundle {
  std::unique_ptr<roborun::RobotBackend> robot;
  roborun::WorkcellBackend* workcell = nullptr;
};

roborun::RuntimeOptions MakeRuntimeOptions(const StationConfiguration& station) {
  roborun::RuntimeOptions options;
  if (station.workcell) {
    options.move_timeout_ms = station.workcell->default_move_timeout_ms;
    options.tool_timeout_ms = station.workcell->default_tool_timeout_ms;
    options.grip_timeout_ms = station.workcell->default_grip_timeout_ms;
    options.workcell_safe_state_timeout_ms = station.workcell->default_safe_state_timeout_ms;
  }
  return options;
}

struct ProgramRequest {
  std::shared_ptr<GoalHandle> goal;
  std::string text;
  std::string source;
};

struct ControlRequest {
  roborun::ControlType type = roborun::ControlType::kNone;
  std::uint64_t request_id = 0;
  std::string source;
  bool action_cancel = false;
};

using BridgeRequest = std::variant<ProgramRequest, ControlRequest>;

std::optional<roborun::ControlType> ParseControl(const std::string& value) {
  if (value == "PAUSE") {
    return roborun::ControlType::kPause;
  }
  if (value == "RESUME") {
    return roborun::ControlType::kResume;
  }
  if (value == "STOP") {
    return roborun::ControlType::kStop;
  }
  if (value == "ESTOP") {
    return roborun::ControlType::kEstop;
  }
  if (value == "RESET") {
    return roborun::ControlType::kReset;
  }
  return std::nullopt;
}

const roborun::Alarm* PrimaryAlarm(const roborun::RuntimeSnapshot& snapshot) {
  if (!snapshot.primary_alarm_index.has_value() ||
      *snapshot.primary_alarm_index >= snapshot.alarms.size()) {
    return nullptr;
  }
  return &snapshot.alarms[*snapshot.primary_alarm_index];
}

std::string PrimaryAlarmCode(const roborun::RuntimeSnapshot& snapshot) {
  const roborun::Alarm* alarm = PrimaryAlarm(snapshot);
  return alarm == nullptr ? "" : roborun::ToString(alarm->code);
}

std::string PrimaryAlarmReason(const roborun::RuntimeSnapshot& snapshot) {
  const roborun::Alarm* alarm = PrimaryAlarm(snapshot);
  return alarm == nullptr ? "" : alarm->reason;
}

class RuntimeWorker {
 public:
  using PublishCallback =
      std::function<void(const roborun::RuntimeSnapshot&, const std::shared_ptr<GoalHandle>&)>;
  using GoalCompleteCallback = std::function<void()>;

  RuntimeWorker(StationConfiguration station, std::unique_ptr<roborun::RobotBackend> backend,
                roborun::WorkcellBackend* workcell, PublishCallback publish,
                GoalCompleteCallback complete)
      : station_(std::move(station)),
        backend_(std::move(backend)),
        clock_(std::make_unique<roborun::VirtualClock>()),
        session_(std::make_unique<roborun::RuntimeSession>(
            backend_.get(), clock_.get(), station_.robot ? &*station_.robot : nullptr,
            station_.points ? &*station_.points : nullptr, station_.io ? &*station_.io : nullptr,
            station_.tools ? &*station_.tools : nullptr,
            station_.scenario ? &*station_.scenario : nullptr, MakeRuntimeOptions(station_),
            workcell)),
        publish_(std::move(publish)),
        complete_(std::move(complete)) {
    worker_ = std::thread([this] { Run(); });
  }

  ~RuntimeWorker() { Shutdown(); }

  bool Enqueue(BridgeRequest request) {
    std::lock_guard lock(mutex_);
    if (stopping_ || requests_.size() == kRequestQueueCapacity) {
      return false;
    }
    const auto* control = std::get_if<ControlRequest>(&request);
    if (control != nullptr && control->type == roborun::ControlType::kEstop) {
      requests_.push_front(std::move(request));
    } else {
      requests_.push_back(std::move(request));
    }
    wake_.notify_one();
    return true;
  }

  void Shutdown() {
    {
      std::lock_guard lock(mutex_);
      if (stopping_) {
        return;
      }
      stopping_ = true;
      if (task_active_.load()) {
        requests_.push_front(
            ControlRequest{roborun::ControlType::kStop, 0, "bridge_shutdown", false});
      }
    }
    wake_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void Run() {
    Publish(session_->Snapshot());
    while (true) {
      std::optional<BridgeRequest> request;
      {
        std::unique_lock lock(mutex_);
        wake_.wait_for(lock, std::chrono::milliseconds(1), [this] {
          return stopping_ || !requests_.empty() ||
                 (task_active_.load() && !session_->is_terminal());
        });
        if (!requests_.empty()) {
          request = std::move(requests_.front());
          requests_.pop_front();
        } else if (stopping_) {
          break;
        }
      }
      const bool processed_control =
          request.has_value() && std::holds_alternative<ControlRequest>(*request);
      if (request.has_value()) {
        Process(*request);
      }
      if (processed_control || (active_goal_ && !session_->is_terminal())) {
        session_->Advance();
        Publish(session_->Snapshot());
        FinishGoalIfTerminal();
      }
    }
    if (active_goal_ && !session_->is_terminal()) {
      session_->SubmitControl(roborun::ControlType::kStop, "bridge_shutdown");
      for (int step = 0; step < kShutdownAdvanceLimit && !session_->is_terminal(); ++step) {
        session_->Advance();
      }
      if (!session_->is_terminal()) {
        AbortForUnconfirmedShutdown();
      }
    }
    Publish(session_->Snapshot());
    FinishGoalIfTerminal();
  }

  void Process(const BridgeRequest& request) {
    if (const auto* program = std::get_if<ProgramRequest>(&request)) {
      StartProgram(*program);
    } else {
      SubmitControl(*std::get_if<ControlRequest>(&request));
    }
  }

  void StartProgram(const ProgramRequest& request) {
    if (active_goal_) {
      return;
    }
    active_goal_ = request.goal;
    task_active_.store(true);
    cancel_requested_ = false;
    std::istringstream input(request.text);
    const roborun::ParseResult parsed = roborun::ParseProgram(input, request.source);
    if (!parsed.diagnostics.empty()) {
      AbortWithDiagnostic(parsed.diagnostics.front());
      return;
    }
    const roborun::ProgramSubmission submission = session_->SubmitProgram(parsed.program);
    Publish(session_->Snapshot());
    if (submission.status == roborun::ProgramSubmissionStatus::kRejected) {
      FinishGoalIfTerminal();
    }
  }

  void SubmitControl(const ControlRequest& request) {
    const roborun::ControlSubmission submission =
        request.request_id == 0 ? session_->SubmitControl(request.type, request.source)
                                : session_->SubmitControlWithRequestId(
                                      request.type, request.request_id, request.source);
    if (request.action_cancel && submission.status == roborun::ControlSubmissionStatus::kQueued) {
      cancel_requested_ = true;
    }
  }

  void AbortWithDiagnostic(const roborun::Diagnostic& diagnostic) {
    const roborun::RuntimeSnapshot snapshot = session_->Snapshot();
    Publish(snapshot);
    auto result = std::make_shared<ExecuteProgram::Result>();
    result->succeeded = false;
    result->outcome = "rejected";
    result->lifecycle = roborun::ToString(snapshot.lifecycle_state);
    result->diagnostic_code = roborun::ToString(diagnostic.code);
    result->diagnostic_message = diagnostic.message;
    result->final_snapshot_sequence = snapshot.sequence;
    active_goal_->abort(result);
    active_goal_.reset();
    task_active_.store(false);
    complete_();
  }

  void AbortForUnconfirmedShutdown() {
    if (!active_goal_) {
      return;
    }
    auto result = std::make_shared<ExecuteProgram::Result>();
    result->succeeded = false;
    result->outcome = "faulted";
    result->lifecycle = roborun::ToString(session_->lifecycle_state());
    result->diagnostic_code = "bridge_shutdown_stop_unconfirmed";
    result->diagnostic_message = "bridge shutdown exceeded its bounded stop wait";
    result->final_snapshot_sequence = session_->Snapshot().sequence;
    active_goal_->abort(result);
    active_goal_.reset();
    task_active_.store(false);
    complete_();
  }

  void FinishGoalIfTerminal() {
    if (!active_goal_ || !session_->is_terminal()) {
      return;
    }
    const roborun::ExecutionResult& execution = session_->result();
    const roborun::RuntimeSnapshot snapshot = session_->Snapshot();
    auto result = std::make_shared<ExecuteProgram::Result>();
    result->succeeded = execution.succeeded;
    result->outcome = roborun::ToString(execution.outcome);
    result->lifecycle = roborun::ToString(execution.lifecycle_state);
    result->final_snapshot_sequence = snapshot.sequence;
    if (!execution.diagnostics.empty()) {
      result->diagnostic_code = roborun::ToString(execution.diagnostics.front().code);
      result->diagnostic_message = execution.diagnostics.front().message;
    }
    if (snapshot.primary_alarm_index && *snapshot.primary_alarm_index < snapshot.alarms.size()) {
      const roborun::Alarm& alarm = snapshot.alarms[*snapshot.primary_alarm_index];
      result->primary_alarm_code = roborun::ToString(alarm.code);
      result->primary_alarm_reason = alarm.reason;
      result->primary_alarm_details = alarm.details;
    }
    if (execution.succeeded) {
      active_goal_->succeed(result);
    } else if (cancel_requested_ &&
               execution.outcome == roborun::ExecutionOutcome::kOperatorStopped) {
      active_goal_->canceled(result);
    } else {
      active_goal_->abort(result);
    }
    active_goal_.reset();
    task_active_.store(false);
    complete_();
    cancel_requested_ = false;
  }

  void Publish(const roborun::RuntimeSnapshot& snapshot) {
    if (has_published_snapshot_ && snapshot.sequence == last_published_sequence_) {
      return;
    }
    has_published_snapshot_ = true;
    last_published_sequence_ = snapshot.sequence;
    publish_(snapshot, active_goal_);
  }

  StationConfiguration station_;
  std::unique_ptr<roborun::RobotBackend> backend_;
  std::unique_ptr<roborun::VirtualClock> clock_;
  std::unique_ptr<roborun::RuntimeSession> session_;
  PublishCallback publish_;
  GoalCompleteCallback complete_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<BridgeRequest> requests_;
  std::shared_ptr<GoalHandle> active_goal_;
  bool cancel_requested_ = false;
  bool stopping_ = false;
  std::atomic<bool> task_active_{false};
  bool has_published_snapshot_ = false;
  std::uint64_t last_published_sequence_ = 0;
};

class BridgeNode final : public rclcpp::Node {
 public:
  BridgeNode() : Node("roborun_bridge") {
    const std::string backend = declare_parameter<std::string>("backend", "mock");
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names", {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                        "wrist_2_joint", "wrist_3_joint"});
    if (joint_names_.size() != roborun::kJointCount) {
      throw std::runtime_error("joint_names must contain exactly six names");
    }
    station_ = LoadStation();
    BackendBundle bundle = BuildBackend(backend);
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
    auto authority_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", sensor_qos);
    status_publisher_ =
        create_publisher<roborun_interfaces::msg::RuntimeStatus>("runtime_status", authority_qos);
    io_publisher_ = create_publisher<roborun_interfaces::msg::IoState>("io_state", authority_qos);
    alarm_publisher_ =
        create_publisher<roborun_interfaces::msg::AlarmState>("alarm_state", authority_qos);
    worker_ = std::make_unique<RuntimeWorker>(
        std::move(station_), std::move(bundle.robot), bundle.workcell,
        [this](const roborun::RuntimeSnapshot& snapshot, const std::shared_ptr<GoalHandle>& goal) {
          PublishSnapshot(snapshot, goal);
        },
        [this] {
          goal_reserved_.store(false);
          cancel_queued_.store(false);
        });
    action_server_ = rclcpp_action::create_server<ExecuteProgram>(
        this, "execute_program",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteProgram::Goal> goal) {
          return AcceptGoal(*goal);
        },
        [this](const std::shared_ptr<GoalHandle>&) { return CancelGoal(); },
        [this](const std::shared_ptr<GoalHandle>& goal) { AcceptGoalHandle(goal); });
    control_service_ = create_service<ControlRuntime>(
        "control_runtime", [this](const ControlRuntime::Request::SharedPtr request,
                                  ControlRuntime::Response::SharedPtr response) {
          SubmitServiceControl(*request, response.get());
        });
  }

  ~BridgeNode() override {
    accepting_goals_.store(false);
    worker_.reset();
  }

 private:
  StationConfiguration LoadStation() {
    StationConfiguration station;
    const std::string robot = declare_parameter<std::string>("robot_config", "");
    const std::string points = declare_parameter<std::string>("points_config", "");
    const std::string io = declare_parameter<std::string>("io_config", "");
    const std::string tools = declare_parameter<std::string>("tool_config", "");
    const std::string scenario = declare_parameter<std::string>("scenario_config", "");
    const std::string workcell = declare_parameter<std::string>("workcell_config", "");
    if (!robot.empty()) {
      const roborun::RobotConfigLoadResult loaded = roborun::LoadRobotConfiguration(robot);
      if (!loaded.diagnostics.empty() || !loaded.configuration) {
        throw std::runtime_error("invalid robot_config");
      }
      station.robot = *loaded.configuration;
    }
    if (!points.empty()) {
      const roborun::PointCatalogLoadResult loaded = roborun::LoadPointCatalog(points);
      if (!loaded.diagnostics.empty() || !loaded.catalog) {
        throw std::runtime_error("invalid points_config");
      }
      station.points = *loaded.catalog;
    }
    if (!io.empty()) {
      const roborun::IOConfigLoadResult loaded = roborun::LoadIOConfiguration(io);
      if (!loaded.diagnostics.empty() || !loaded.configuration) {
        throw std::runtime_error("invalid io_config");
      }
      station.io = *loaded.configuration;
    }
    if (!tools.empty()) {
      const roborun::ToolConfigLoadResult loaded = roborun::LoadToolConfiguration(tools);
      if (!loaded.diagnostics.empty() || !loaded.configuration) {
        throw std::runtime_error("invalid tool_config");
      }
      station.tools = *loaded.configuration;
    }
    if (!scenario.empty()) {
      const roborun::ScenarioLoadResult loaded = roborun::LoadMockScenario(scenario);
      if (!loaded.diagnostics.empty() || !loaded.scenario) {
        throw std::runtime_error("invalid scenario_config");
      }
      station.scenario = *loaded.scenario;
    }
    if (!workcell.empty()) {
      const roborun::WorkcellConfigLoadResult loaded = roborun::LoadWorkcellConfiguration(workcell);
      if (!loaded.diagnostics.empty() || !loaded.configuration) {
        throw std::runtime_error("invalid workcell_config");
      }
      station.workcell = *loaded.configuration;
    }
    if (station.robot && station.points) {
      const std::vector<roborun::Diagnostic> diagnostics =
          roborun::ValidatePointCatalog(*station.points, *station.robot);
      if (!diagnostics.empty()) {
        throw std::runtime_error("invalid points_config for robot_config");
      }
    }
    return station;
  }

  BackendBundle BuildBackend(const std::string& backend) {
    if (backend == "mock") {
      if (station_.scenario) {
        return {std::make_unique<roborun::ScriptedMockBackend>(*station_.scenario), nullptr};
      }
      return {std::make_unique<roborun::MockRobotBackend>(), nullptr};
    }
#ifdef ROBORUN_WITH_COPPELIASIM
    if (backend == "coppeliasim") {
      roborun::CoppeliaSimConfig configuration;
      configuration.host = declare_parameter<std::string>("coppeliasim_host", "127.0.0.1");
      configuration.port = declare_parameter<int>("coppeliasim_port", 23000);
      configuration.model_path = declare_parameter<std::string>("model_path", "");
      configuration.resources_path = declare_parameter<std::string>("resources_path", "");
      configuration.scene_path = declare_parameter<std::string>("scene_path", "");
      configuration.workcell = station_.workcell;
      auto robot = std::make_unique<roborun::CoppeliaSimBackend>(std::move(configuration));
      roborun::WorkcellBackend* workcell = station_.workcell ? robot.get() : nullptr;
      return {std::move(robot), workcell};
    }
#endif
    throw std::runtime_error("backend must be mock or an enabled coppeliasim backend");
  }

  rclcpp_action::GoalResponse AcceptGoal(const ExecuteProgram::Goal& goal) {
    if (!accepting_goals_.load() || goal.program_text.empty() ||
        goal.program_text.size() > kMaxProgramBytes ||
        goal.source_name.size() > kMaxSourceNameBytes || goal_reserved_.exchange(true))
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse CancelGoal() {
    if (!goal_reserved_.load()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    if (cancel_queued_.exchange(true)) {
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    if (!worker_->Enqueue(ControlRequest{roborun::ControlType::kStop, NextControlRequestId(),
                                         "action_cancel", true})) {
      cancel_queued_.store(false);
      return rclcpp_action::CancelResponse::REJECT;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void AcceptGoalHandle(const std::shared_ptr<GoalHandle>& goal) {
    if (!worker_->Enqueue(
            ProgramRequest{goal, goal->get_goal()->program_text, goal->get_goal()->source_name})) {
      goal_reserved_.store(false);
      auto result = std::make_shared<ExecuteProgram::Result>();
      result->diagnostic_code = "BRIDGE_QUEUE_FULL";
      result->diagnostic_message = "bridge request queue is full";
      goal->abort(result);
    }
  }

  void SubmitServiceControl(const ControlRuntime::Request& request,
                            ControlRuntime::Response* response) {
    response->control = request.control;
    const auto control = ParseControl(request.control);
    if (!control) {
      response->status = "rejected";
      response->message = "unknown control";
      return;
    }
    const std::uint64_t request_id = NextControlRequestId();
    if (!worker_->Enqueue(ControlRequest{*control, request_id, request.source, false})) {
      response->status = "rejected";
      response->message = "bridge request queue is unavailable";
      return;
    }
    response->request_id = request_id;
    response->status = "queued";
    response->message = "";
  }

  std::uint64_t NextControlRequestId() { return next_control_request_id_.fetch_add(1); }

  void PublishSnapshot(const roborun::RuntimeSnapshot& snapshot,
                       const std::shared_ptr<GoalHandle>& goal) {
    const rclcpp::Time now = get_clock()->now();
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now;
    joints.name = joint_names_;
    joints.position.assign(snapshot.joint_positions.begin(), snapshot.joint_positions.end());
    joint_publisher_->publish(joints);

    roborun_interfaces::msg::RuntimeStatus status;
    status.stamp = now;
    status.snapshot_sequence = snapshot.sequence;
    status.business_time_ms = snapshot.business_time_ms;
    status.lifecycle = roborun::ToString(snapshot.lifecycle_state);
    status.runtime_state = roborun::ToString(snapshot.runtime_state);
    status.backend_state = roborun::ToString(snapshot.backend_state);
    status.backend_operation_state = roborun::ToString(snapshot.backend_operation_state);
    status.terminal = snapshot.terminal.has_value();
    status.outcome = snapshot.terminal ? roborun::ToString(snapshot.terminal->outcome) : "none";
    if (snapshot.active_command) {
      status.active_command_index = snapshot.active_command->command_index;
      status.active_command_line = snapshot.active_command->line;
      status.active_command_type = roborun::ToString(snapshot.active_command->type);
      status.active_command_status = roborun::ToString(snapshot.active_command->status);
    }
    status.primary_alarm_code = PrimaryAlarmCode(snapshot);
    status.primary_alarm_reason = PrimaryAlarmReason(snapshot);
    status_publisher_->publish(status);

    roborun_interfaces::msg::IoState io;
    io.stamp = now;
    io.snapshot_sequence = snapshot.sequence;
    io.business_time_ms = snapshot.business_time_ms;
    for (const auto& [name, signal] : snapshot.signals) {
      roborun_interfaces::msg::SignalState message;
      message.name = name;
      message.direction = roborun::ToString(signal.direction);
      message.value = signal.value;
      message.last_update_business_time_ms = signal.last_update_ms;
      io.signals.push_back(std::move(message));
    }
    for (const auto& [name, tool] : snapshot.tools) {
      roborun_interfaces::msg::ToolState message;
      message.name = name;
      message.type = roborun::ToString(tool.type);
      message.commanded = roborun::ToString(tool.commanded);
      message.observed = roborun::ToString(tool.observed);
      message.last_feedback_business_time_ms = tool.last_feedback_ms;
      io.tools.push_back(std::move(message));
    }
    io_publisher_->publish(io);

    roborun_interfaces::msg::AlarmState alarms;
    alarms.stamp = now;
    alarms.snapshot_sequence = snapshot.sequence;
    alarms.business_time_ms = snapshot.business_time_ms;
    alarms.primary_alarm_index = snapshot.primary_alarm_index ? *snapshot.primary_alarm_index : -1;
    alarms.primary_alarm_code = PrimaryAlarmCode(snapshot);
    for (const roborun::Alarm& alarm : snapshot.alarms) {
      roborun_interfaces::msg::AlarmStateEntry message;
      message.code = roborun::ToString(alarm.code);
      message.severity = roborun::ToString(alarm.severity);
      message.source = alarm.source;
      message.reason = alarm.reason;
      message.details = alarm.details;
      message.raised_business_time_ms = alarm.raised_business_time_ms;
      message.recoverable = alarm.recoverable;
      message.active = alarm.active;
      alarms.alarms.push_back(std::move(message));
    }
    alarm_publisher_->publish(alarms);

    if (goal && !snapshot.terminal) {
      auto feedback = std::make_shared<ExecuteProgram::Feedback>();
      feedback->snapshot_sequence = snapshot.sequence;
      feedback->business_time_ms = snapshot.business_time_ms;
      feedback->lifecycle = status.lifecycle;
      feedback->runtime_state = status.runtime_state;
      feedback->backend_state = status.backend_state;
      feedback->backend_operation_state = status.backend_operation_state;
      feedback->active_command_index = status.active_command_index;
      feedback->active_command_line = status.active_command_line;
      feedback->active_command_type = status.active_command_type;
      feedback->active_command_status = status.active_command_status;
      feedback->primary_alarm_code = status.primary_alarm_code;
      feedback->primary_alarm_reason = status.primary_alarm_reason;
      goal->publish_feedback(feedback);
    }
  }

  StationConfiguration station_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<RuntimeWorker> worker_;
  std::atomic<bool> accepting_goals_{true};
  std::atomic<bool> goal_reserved_{false};
  std::atomic<bool> cancel_queued_{false};
  std::atomic<std::uint64_t> next_control_request_id_{1};
  rclcpp_action::Server<ExecuteProgram>::SharedPtr action_server_;
  rclcpp::Service<ControlRuntime>::SharedPtr control_service_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<roborun_interfaces::msg::RuntimeStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<roborun_interfaces::msg::IoState>::SharedPtr io_publisher_;
  rclcpp::Publisher<roborun_interfaces::msg::AlarmState>::SharedPtr alarm_publisher_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
