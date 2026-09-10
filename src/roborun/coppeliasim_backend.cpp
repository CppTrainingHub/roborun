#include "roborun/coppeliasim_backend.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "RemoteAPIClient.h"
#include "roborun/coppeliasim_motion.h"

namespace roborun {
namespace {

constexpr double kTcpOffsetMeters = 0.19;
constexpr double kToolJointStep = 0.005;

std::string AliasLeaf(const std::string& alias) {
  const std::size_t separator = alias.find_last_of('/');
  return separator == std::string::npos ? alias : alias.substr(separator + 1);
}

int64_t ExpectedObjectType(const RemoteAPIObject::sim& sim, const std::string& type) {
  if (type == "dummy") {
    return sim.object_dummy_type;
  }
  if (type == "joint") {
    return sim.object_joint_type;
  }
  if (type == "shape") {
    return sim.object_shape_type;
  }
  if (type == "model") {
    return sim.object_shape_type;
  }
  return -1;
}

Pose ToPose(const std::vector<double>& values) {
  if (values.size() != Pose{}.size()) {
    throw std::runtime_error("CoppeliaSim returned a malformed pose");
  }
  Pose pose{};
  std::copy(values.begin(), values.end(), pose.begin());
  return pose;
}

std::vector<double> ToVector(const Pose& pose) {
  return std::vector<double>(pose.begin(), pose.end());
}

int64_t FindDescendantByAlias(RemoteAPIObject::sim* sim, int64_t root, const std::string& alias) {
  for (const int64_t handle : sim->getObjectsInTree(root)) {
    if (sim->getObjectAlias(handle) == alias) {
      return handle;
    }
  }
  throw std::runtime_error("CoppeliaSim model is missing object alias: " + alias);
}

bool IsDescendantOf(RemoteAPIObject::sim* sim, int64_t object, int64_t ancestor) {
  for (int64_t parent = sim->getObjectParent(object); parent != -1;
       parent = sim->getObjectParent(parent)) {
    if (parent == ancestor) {
      return true;
    }
  }
  return false;
}

void RemoveModelScripts(RemoteAPIObject::sim* sim, int64_t model) {
  const std::vector<int64_t> scripts = sim->getObjectsInTree(model, sim->object_script_type);
  if (!scripts.empty()) {
    sim->removeObjects(scripts);
  }
}

void MakeModelKinematic(RemoteAPIObject::sim* sim, int64_t model) {
  for (const int64_t shape : sim->getObjectsInTree(model, sim->object_shape_type)) {
    sim->setBoolProperty(shape, "dynamic", false);
  }
}

}  // namespace

struct CoppeliaSimBackend::Impl {
  std::unique_ptr<RemoteAPIClient> client;
  std::unique_ptr<RemoteAPIObject::sim> sim;
  std::vector<int64_t> joint_handles;
  bool simulation_running = false;
  bool motion_active = false;
  bool stop_active = false;
  StopReason stop_reason = StopReason::kProgramComplete;
  JointPositions motion_initial{};
  JointPositions motion_target{};
  int motion_steps = 0;
  int motion_step = 0;
  int motion_stable_samples = 0;
  std::optional<WorkcellConfiguration> workcell;
  int64_t robot_handle = -1;
  int64_t robot_flange_handle = -1;
  int64_t workcell_root_handle = -1;
  int64_t tool_mount_handle = -1;
  int64_t tool_handle = -1;
  int64_t tool_joint_handle = -1;
  int64_t tool_motor_joint_handle = -1;
  int64_t tcp_handle = -1;
  int64_t workpiece_handle = -1;
  int64_t pick_handle = -1;
  int64_t place_handle = -1;
  std::map<std::string, DigitalSignal> signals;
  std::map<std::string, ToolState> tools;
  std::map<std::string, int> tool_stable_samples;
  std::optional<std::pair<std::string, ToolPosition>> pending_tool_command;
  double tool_motor_target = 0.05;
  int attached_stable_samples = 0;
  int place_stable_samples = 0;
  int placement_failure_samples = 0;
  bool attachment_requested = false;
  bool release_requested = false;
  bool safe_requested = false;
  bool cycle_advanced_by_motion = false;
  std::uint64_t workcell_generation = 0;

  void ResetSessionState() {
    sim.reset();
    client.reset();
    joint_handles.clear();
    simulation_running = false;
    motion_active = false;
    stop_active = false;
    stop_reason = StopReason::kProgramComplete;
    motion_initial = {};
    motion_target = {};
    motion_steps = 0;
    motion_step = 0;
    motion_stable_samples = 0;
    workcell.reset();
    robot_handle = -1;
    robot_flange_handle = -1;
    workcell_root_handle = -1;
    tool_mount_handle = -1;
    tool_handle = -1;
    tool_joint_handle = -1;
    tool_motor_joint_handle = -1;
    tcp_handle = -1;
    workpiece_handle = -1;
    pick_handle = -1;
    place_handle = -1;
    signals.clear();
    tools.clear();
    tool_stable_samples.clear();
    pending_tool_command.reset();
    tool_motor_target = 0.05;
    attached_stable_samples = 0;
    place_stable_samples = 0;
    placement_failure_samples = 0;
    attachment_requested = false;
    release_requested = false;
    safe_requested = false;
    cycle_advanced_by_motion = false;
    workcell_generation = 0;
  }
};

CoppeliaSimBackend::CoppeliaSimBackend(CoppeliaSimConfig config)
    : config_(std::move(config)), impl_(std::make_unique<Impl>()) {}

CoppeliaSimBackend::~CoppeliaSimBackend() = default;

std::string CoppeliaSimBackend::Name() const { return "coppeliasim"; }

std::string CoppeliaSimBackend::Version() const { return "CoppeliaSim ZeroMQ Remote API"; }

void CoppeliaSimBackend::Connect() {
  if (config_.model_path.empty() && !config_.workcell.has_value()) {
    throw std::runtime_error("CoppeliaSim UR5 model path is required");
  }

  impl_->ResetSessionState();
  servo_enabled_ = false;

  impl_->client = std::make_unique<RemoteAPIClient>(config_.host, config_.port);
  impl_->sim = std::make_unique<RemoteAPIObject::sim>(impl_->client.get());
  int64_t robot_handle = -1;
  if (config_.workcell.has_value()) {
    impl_->workcell = config_.workcell;
    if (impl_->sim->getSimulationState() != impl_->sim->simulation_stopped) {
      impl_->sim->stopSimulation(true);
    }
    if (!config_.scene_path.empty() && std::filesystem::exists(config_.scene_path)) {
      impl_->sim->loadScene(config_.scene_path);
    } else {
      BuildWorkcell();
    }
    ValidateConnectedWorkcell();
    robot_handle = impl_->sim->getObject(impl_->workcell->robot_alias);
    impl_->robot_handle = robot_handle;
    impl_->robot_flange_handle =
        FindDescendantByAlias(impl_->sim.get(), robot_handle, "connection");
    for (const auto& [name, mapping] : impl_->workcell->tools) {
      impl_->tools.emplace(
          name, ToolState{name, ToolType::kGripper, ToolPosition::kOpen, ToolPosition::kOpen, 0});
      impl_->tool_stable_samples.emplace(name, 0);
      impl_->tool_joint_handle = impl_->sim->getObject(mapping.joint_alias);
      impl_->tcp_handle = impl_->sim->getObject(mapping.tcp_alias);
      impl_->tool_motor_joint_handle =
          FindDescendantByAlias(impl_->sim.get(), impl_->tool_handle, "openCloseJoint");
    }
    for (const auto& [name, mapping] : impl_->workcell->signals) {
      impl_->signals.emplace(name, mapping.signal);
      const bool observed = impl_->sim->getBoolProperty(impl_->sim->handle_scene, mapping.property);
      impl_->signals.at(name).value = observed;
    }
  } else {
    robot_handle = impl_->sim->loadModel(config_.model_path);
  }
  impl_->joint_handles = impl_->sim->getObjectsInTree(robot_handle, impl_->sim->object_joint_type);
  if (impl_->workcell.has_value()) {
    std::erase_if(impl_->joint_handles, [&](int64_t handle) {
      return IsDescendantOf(impl_->sim.get(), handle, impl_->tool_mount_handle);
    });
  }
  if (impl_->joint_handles.size() != kJointCount) {
    throw std::runtime_error("UR5 model did not expose exactly 6 joints");
  }

  for (const int64_t joint_handle : impl_->joint_handles) {
    impl_->sim->setJointMode(joint_handle, impl_->sim->jointmode_kinematic);
  }
  impl_->sim->setStepping(true);
  impl_->sim->startSimulation();
  impl_->simulation_running = true;
  // Let the packaged UR5 initialize its script-controlled home configuration before a request.
  impl_->sim->step();
  UpdateToolMount();
}

void CoppeliaSimBackend::UpdateToolMount() {
  if (impl_->sim == nullptr || impl_->robot_flange_handle == -1 || impl_->tool_mount_handle == -1) {
    return;
  }
  impl_->sim->setObjectPose(impl_->tool_mount_handle,
                            impl_->sim->getObjectPose(impl_->robot_flange_handle));
}

void CoppeliaSimBackend::ValidateConnectedWorkcell() {
  if (!impl_->workcell.has_value() || impl_->sim == nullptr) {
    throw std::runtime_error("CoppeliaSim workcell validation requires a connected workcell");
  }
  const WorkcellManifest manifest = MakeWorkcellManifest(*impl_->workcell);
  const auto [schema_bytes, ignored_tag] = impl_->sim->readCustomDataBlockEx(
      impl_->sim->getObject(impl_->workcell->root_alias), "customData.roborun.schema");
  (void)ignored_tag;
  const std::string observed_schema(schema_bytes.begin(), schema_bytes.end());
  if (observed_schema != impl_->workcell->scene_schema) {
    throw std::runtime_error("workcell scene schema does not match the configured schema");
  }
  const auto [manifest_bytes, ignored_manifest_tag] = impl_->sim->readCustomDataBlockEx(
      impl_->sim->getObject(impl_->workcell->root_alias), "customData.roborun.manifest");
  (void)ignored_manifest_tag;
  const std::string observed_manifest(manifest_bytes.begin(), manifest_bytes.end());
  if (observed_manifest != SerializeWorkcellManifest(manifest)) {
    throw std::runtime_error("workcell scene manifest does not match the configured manifest");
  }
  for (const WorkcellObject& object : manifest.objects) {
    const int64_t handle = impl_->sim->getObject(object.alias);
    const int64_t expected_type = ExpectedObjectType(*impl_->sim, object.type);
    if (expected_type != -1 && impl_->sim->getObjectType(handle) != expected_type) {
      throw std::runtime_error("workcell object type does not match manifest: " + object.alias);
    }
    if (!object.parent_alias.empty()) {
      const int64_t expected_parent = impl_->sim->getObject(object.parent_alias);
      if (impl_->sim->getObjectParent(handle) != expected_parent) {
        throw std::runtime_error("workcell object parent does not match manifest: " + object.alias);
      }
    }
  }
  for (const auto& [name, mapping] : impl_->workcell->signals) {
    // Scene signal properties are volatile across save/load. The persistent manifest above is
    // their validated source, so recreate the initial value and immediately read it back.
    impl_->sim->setBoolProperty(impl_->sim->handle_scene, mapping.property, mapping.signal.value);
    const bool observed = impl_->sim->getBoolProperty(impl_->sim->handle_scene, mapping.property);
    if (observed != mapping.signal.value) {
      throw std::runtime_error("workcell initial I/O does not match manifest: " + name);
    }
  }
  for (const auto& [name, mapping] : impl_->workcell->tools) {
    impl_->sim->setIntProperty(impl_->sim->handle_scene, mapping.command_property, 1);
    if (impl_->sim->getIntProperty(impl_->sim->handle_scene, mapping.command_property) != 1) {
      throw std::runtime_error("workcell initial tool command does not match manifest: " + name);
    }
    const int64_t tool_handle = impl_->sim->getObject(mapping.alias);
    const int64_t motor_handle =
        FindDescendantByAlias(impl_->sim.get(), tool_handle, "openCloseJoint");
    if (std::abs(impl_->sim->getJointPosition(motor_handle) - mapping.open_joint_position) >
        mapping.aperture_tolerance) {
      throw std::runtime_error("workcell initial tool pose does not match manifest: " + name);
    }
  }
  const auto validate_pose = [&](const std::string& alias, const Pose& expected) {
    const Pose observed = ToPose(impl_->sim->getObjectPose(impl_->sim->getObject(alias)));
    if (!IsPoseWithinTolerance(observed, expected, 1e-3, 1e-3)) {
      throw std::runtime_error("workcell object pose does not match manifest: " + alias);
    }
  };
  validate_pose(impl_->workcell->workpiece_alias, impl_->workcell->workpiece_initial_pose);
  validate_pose(impl_->workcell->pick_alias, impl_->workcell->pick_pose);
  validate_pose(impl_->workcell->place_alias, impl_->workcell->place_pose);
  impl_->workcell_root_handle = impl_->sim->getObject(impl_->workcell->root_alias);
  impl_->tool_mount_handle = impl_->sim->getObject(impl_->workcell->tool_mount_alias);
  impl_->tool_handle = impl_->sim->getObject(impl_->workcell->tool_alias);
  impl_->workpiece_handle = impl_->sim->getObject(impl_->workcell->workpiece_alias);
  impl_->pick_handle = impl_->sim->getObject(impl_->workcell->pick_alias);
  impl_->place_handle = impl_->sim->getObject(impl_->workcell->place_alias);
}

void CoppeliaSimBackend::BuildWorkcell() {
  if (!impl_->workcell.has_value() || config_.resources_path.empty()) {
    throw std::runtime_error("CoppeliaSim workcell requires resources_path and configuration");
  }
  const WorkcellConfiguration& configuration = *impl_->workcell;
  impl_->sim->closeScene();
  impl_->workcell_root_handle = impl_->sim->createDummy(0.01);
  impl_->sim->setObjectAlias(impl_->workcell_root_handle, AliasLeaf(configuration.root_alias));
  const std::string ur5_path = config_.resources_path + "/models/robots/non-mobile/UR5.ttm";
  const std::string rg2_path = config_.resources_path + "/models/components/grippers/RG2.ttm";
  const int64_t robot_handle = impl_->sim->loadModel(ur5_path);
  RemoveModelScripts(impl_->sim.get(), robot_handle);
  MakeModelKinematic(impl_->sim.get(), robot_handle);
  impl_->sim->setObjectAlias(robot_handle, AliasLeaf(configuration.robot_alias));
  impl_->sim->setObjectParent(robot_handle, impl_->workcell_root_handle, false);
  impl_->sim->setObjectPose(robot_handle, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                            impl_->workcell_root_handle);
  impl_->robot_handle = robot_handle;
  impl_->robot_flange_handle = FindDescendantByAlias(impl_->sim.get(), robot_handle, "connection");
  impl_->tool_mount_handle = impl_->sim->createDummy(0.01);
  impl_->sim->setObjectAlias(impl_->tool_mount_handle, AliasLeaf(configuration.tool_mount_alias));
  impl_->sim->setObjectParent(impl_->tool_mount_handle, robot_handle, true);
  impl_->sim->setObjectPose(impl_->tool_mount_handle,
                            impl_->sim->getObjectPose(impl_->robot_flange_handle));
  impl_->tool_handle = impl_->sim->loadModel(rg2_path);
  RemoveModelScripts(impl_->sim.get(), impl_->tool_handle);
  MakeModelKinematic(impl_->sim.get(), impl_->tool_handle);
  impl_->sim->setObjectAlias(impl_->tool_handle, AliasLeaf(configuration.tool_alias));
  impl_->sim->setObjectParent(impl_->tool_handle, impl_->tool_mount_handle, false);
  impl_->sim->setObjectPose(impl_->tool_handle, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                            impl_->tool_mount_handle);
  impl_->tool_motor_joint_handle =
      FindDescendantByAlias(impl_->sim.get(), impl_->tool_handle, "openCloseJoint");
  impl_->sim->setJointMode(impl_->tool_motor_joint_handle, impl_->sim->jointmode_kinematic);
  impl_->tool_joint_handle =
      impl_->sim->createJoint(impl_->sim->joint_prismatic_subtype, impl_->sim->jointmode_kinematic,
                              0, std::vector<double>{0.005, 0.01});
  impl_->sim->setObjectAlias(impl_->tool_joint_handle, "Joint");
  impl_->sim->setObjectParent(impl_->tool_joint_handle, impl_->tool_handle, false);
  impl_->sim->setJointInterval(impl_->tool_joint_handle, false, {0.0, 0.1});
  impl_->tcp_handle = impl_->sim->createDummy(0.01);
  impl_->sim->setObjectAlias(impl_->tcp_handle, "TCP");
  impl_->sim->setObjectParent(impl_->tcp_handle, impl_->tool_handle, false);
  impl_->sim->setObjectPose(impl_->tcp_handle, {0.0, 0.0, kTcpOffsetMeters, 0.0, 0.0, 0.0, 1.0},
                            impl_->tool_handle);
  impl_->workpiece_handle =
      impl_->sim->createPrimitiveShape(impl_->sim->primitiveshape_cuboid, {0.05, 0.05, 0.05});
  impl_->sim->setObjectAlias(impl_->workpiece_handle, AliasLeaf(configuration.workpiece_alias));
  impl_->sim->setObjectParent(impl_->workpiece_handle, impl_->workcell_root_handle, true);
  impl_->sim->setObjectPose(impl_->workpiece_handle,
                            ToVector(configuration.workpiece_initial_pose));
  impl_->pick_handle = impl_->sim->createDummy(0.02);
  impl_->sim->setObjectAlias(impl_->pick_handle, AliasLeaf(configuration.pick_alias));
  impl_->sim->setObjectParent(impl_->pick_handle, impl_->workcell_root_handle, true);
  impl_->sim->setObjectPose(impl_->pick_handle, ToVector(configuration.pick_pose));
  impl_->place_handle = impl_->sim->createDummy(0.02);
  impl_->sim->setObjectAlias(impl_->place_handle, AliasLeaf(configuration.place_alias));
  impl_->sim->setObjectParent(impl_->place_handle, impl_->workcell_root_handle, true);
  impl_->sim->setObjectPose(impl_->place_handle, ToVector(configuration.place_pose));
  const std::vector<uint8_t> schema(configuration.scene_schema.begin(),
                                    configuration.scene_schema.end());
  impl_->sim->writeCustomDataBlockEx(impl_->workcell_root_handle, "customData.roborun.schema",
                                     schema);
  const std::string manifest = SerializeWorkcellManifest(MakeWorkcellManifest(configuration));
  impl_->sim->writeCustomDataBlockEx(impl_->workcell_root_handle, "customData.roborun.manifest",
                                     std::vector<uint8_t>(manifest.begin(), manifest.end()));
  for (const auto& [name, mapping] : configuration.signals) {
    impl_->sim->setBoolProperty(impl_->sim->handle_scene, mapping.property, mapping.signal.value);
    (void)name;
  }
  for (const auto& [name, mapping] : configuration.tools) {
    impl_->sim->setIntProperty(impl_->sim->handle_scene, mapping.command_property, 1);
    impl_->sim->setJointPosition(impl_->tool_motor_joint_handle, mapping.open_joint_position);
    impl_->tool_motor_target = mapping.open_joint_position;
    (void)name;
  }
  if (!config_.scene_path.empty()) {
    impl_->sim->saveScene(config_.scene_path);
  }
}

void CoppeliaSimBackend::SetServoEnabled(bool enabled) { servo_enabled_ = enabled; }

MotionSubmission CoppeliaSimBackend::SubmitMoveJ(const MoveJRequest& request) {
  const JointPositions& targets = request.targets;
  if (request.speed_percent < 1 || request.speed_percent > 100) {
    throw std::runtime_error("CoppeliaSim backend requires speed_percent in the range 1 to 100");
  }
  if (!std::isfinite(request.joint_speed_radians_per_second) ||
      request.joint_speed_radians_per_second <= 0.0) {
    throw std::runtime_error("CoppeliaSim backend requires a positive finite joint speed");
  }
  if (!servo_enabled_) {
    throw std::runtime_error("CoppeliaSim backend requires an enabled servo");
  }
  if (!impl_->simulation_running) {
    throw std::runtime_error("CoppeliaSim simulation is not running");
  }

  if (impl_->motion_active) {
    throw std::runtime_error("CoppeliaSim backend already has an active motion");
  }

  impl_->motion_initial = ReadJointPositions();
  impl_->motion_target = targets;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    impl_->sim->setJointTargetPosition(impl_->joint_handles[index], targets[index]);
    const double reported_target = impl_->sim->getJointTargetPosition(impl_->joint_handles[index]);
    if (std::abs(reported_target - targets[index]) > 0.001) {
      throw std::runtime_error("CoppeliaSim joint " + std::to_string(index + 1) +
                               " rejected its target position");
    }
  }

  // The installed UR5 model has no controller contract for target-position commands.
  // Interpolation keeps MOVEJ deterministic while every state change still occurs remotely.
  impl_->motion_steps = ComputeCoppeliaSimMotionSteps(impl_->motion_initial, impl_->motion_target,
                                                      request.joint_speed_radians_per_second,
                                                      impl_->sim->getSimulationTimeStep());
  impl_->motion_step = 0;
  impl_->motion_stable_samples = 0;
  impl_->motion_active = true;
  return MotionSubmission{MotionSubmissionStatus::kAccepted, "", 0};
}

MotionSnapshot CoppeliaSimBackend::PollMotion() {
  if (!impl_->motion_active) {
    throw std::runtime_error("CoppeliaSim backend has no submitted motion");
  }
  impl_->sim->step();
  impl_->cycle_advanced_by_motion = true;
  ++impl_->motion_step;
  const double progress = std::min(
      1.0, static_cast<double>(impl_->motion_step) / static_cast<double>(impl_->motion_steps));
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const double position = impl_->motion_initial[index] +
                            progress * (impl_->motion_target[index] - impl_->motion_initial[index]);
    impl_->sim->setJointPosition(impl_->joint_handles[index], position);
  }
  UpdateToolMount();
  const JointPositions actual = ReadJointPositions();
  const double tolerance =
      impl_->workcell.has_value() ? impl_->workcell->joint_tolerance_radians : 0.02;
  bool in_tolerance = true;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    in_tolerance =
        in_tolerance && std::abs(actual[index] - impl_->motion_target[index]) <= tolerance;
  }
  impl_->motion_stable_samples = in_tolerance ? impl_->motion_stable_samples + 1 : 0;
  const int required_stable_samples =
      impl_->workcell.has_value() ? impl_->workcell->stable_steps : 1;
  if (impl_->motion_step < impl_->motion_steps ||
      impl_->motion_stable_samples < required_stable_samples) {
    return MotionSnapshot{MotionStatus::kRunning, actual, impl_->motion_step, "", 0};
  }
  impl_->motion_active = false;
  return MotionSnapshot{MotionStatus::kSucceeded, actual, impl_->motion_step, "", std::nullopt};
}

JointPositions CoppeliaSimBackend::ReadJointPositions() const {
  if (impl_->sim == nullptr || impl_->joint_handles.size() != kJointCount) {
    throw std::runtime_error("CoppeliaSim backend is not connected");
  }

  JointPositions positions{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    positions[index] = impl_->sim->getJointPosition(impl_->joint_handles[index]);
  }
  return positions;
}

void CoppeliaSimBackend::StopSimulation() {
  servo_enabled_ = false;
  impl_->motion_active = false;
  impl_->stop_active = false;
  if (impl_->sim != nullptr && impl_->simulation_running) {
    impl_->sim->stopSimulation(true);
    impl_->simulation_running = false;
  }
}

void CoppeliaSimBackend::QuitSimulator() {
  RemoteAPIClient client(config_.host, config_.port);
  RemoteAPIObject::sim sim(&client);
  sim.quitSimulator();
}

StopSubmission CoppeliaSimBackend::SubmitStop(StopReason reason) {
  try {
    if (impl_->sim == nullptr || !impl_->simulation_running) {
      return {StopSubmissionStatus::kRejected, "CoppeliaSim simulation is not running",
              std::nullopt};
    }
    // lifecycle control stops the robot motion request, not the simulator process. CoppeliaSim owns
    // the wider simulation/session/process shutdown boundary.
    impl_->motion_active = false;
    impl_->stop_active = true;
    impl_->stop_reason = reason;
    return {StopSubmissionStatus::kAccepted, "", 0};
  } catch (const std::exception& error) {
    return {StopSubmissionStatus::kRejected, error.what(), std::nullopt};
  }
}

StopSnapshot CoppeliaSimBackend::PollStop() {
  try {
    if (!impl_->stop_active || impl_->sim == nullptr || !impl_->simulation_running) {
      return {StopStatus::kFailed, {}, 0, "CoppeliaSim stop was not submitted", std::nullopt};
    }
    const JointPositions positions = ReadJointPositions();
    impl_->stop_active = false;
    return {StopStatus::kStopped, positions, 1, "", std::nullopt};
  } catch (const std::exception& error) {
    return {StopStatus::kFailed, {}, 0, error.what(), std::nullopt};
  }
}

void CoppeliaSimBackend::Disconnect() {
  // The runtime owns only the transport release. Simulation stop and process exit are
  // explicit caller operations so STOP/ESTOP never silently terminate the simulator.
  impl_->ResetSessionState();
  servo_enabled_ = false;
}

WorkcellCommandSubmission CoppeliaSimBackend::SubmitToolCommand(const std::string& name,
                                                                ToolPosition position) {
  if (!impl_->workcell.has_value() || !impl_->tools.contains(name)) {
    return {WorkcellCommandStatus::kRejected, "unknown CoppeliaSim workcell tool", std::nullopt};
  }
  const WorkcellToolConfiguration& mapping = impl_->workcell->tools.at(name);
  try {
    impl_->sim->setIntProperty(impl_->sim->handle_scene, mapping.command_property,
                               position == ToolPosition::kOpen ? 1 : 0);
  } catch (const std::exception& error) {
    return {WorkcellCommandStatus::kRejected, error.what(), std::nullopt};
  }
  impl_->tools.at(name).commanded = position;
  impl_->tool_motor_target =
      position == ToolPosition::kOpen ? mapping.open_joint_position : mapping.closed_joint_position;
  impl_->tool_stable_samples.at(name) = 0;
  impl_->pending_tool_command = std::make_pair(name, position);
  impl_->attachment_requested = false;
  impl_->release_requested = false;
  return {WorkcellCommandStatus::kAccepted, "", 0};
}

WorkcellCommandSubmission CoppeliaSimBackend::SetDigitalOutput(const std::string& name,
                                                               bool value) {
  if (!impl_->workcell.has_value() || !impl_->signals.contains(name) ||
      impl_->signals.at(name).direction != SignalDirection::kDo) {
    return {WorkcellCommandStatus::kRejected, "unknown CoppeliaSim digital output", std::nullopt};
  }
  try {
    const WorkcellSignalConfiguration& mapping = impl_->workcell->signals.at(name);
    impl_->sim->setBoolProperty(impl_->sim->handle_scene, mapping.property, value);
  } catch (const std::exception& error) {
    return {WorkcellCommandStatus::kRejected, error.what(), std::nullopt};
  }
  return {WorkcellCommandStatus::kAccepted, "", 0};
}

WorkcellSnapshot CoppeliaSimBackend::ReadWorkcellSnapshot() {
  WorkcellSnapshot snapshot;
  if (!impl_->workcell.has_value() || impl_->sim == nullptr) {
    snapshot.message = "CoppeliaSim workcell is not connected";
    return snapshot;
  }
  snapshot.valid = true;
  snapshot.generation = ++impl_->workcell_generation;
  snapshot.scene_schema = impl_->workcell->scene_schema;
  snapshot.joint_positions = ReadJointPositions();
  snapshot.required_stable_samples = impl_->workcell->stable_steps;
  snapshot.simulation_running = impl_->simulation_running;
  Pose workpiece_pose = ToPose(impl_->sim->getObjectPose(impl_->workpiece_handle));
  const Pose pick_pose = ToPose(impl_->sim->getObjectPose(impl_->pick_handle));
  const Pose place_pose = ToPose(impl_->sim->getObjectPose(impl_->place_handle));
  const Pose tcp_pose = ToPose(impl_->sim->getObjectPose(impl_->tcp_handle));
  for (const auto& [name, mapping] : impl_->workcell->signals) {
    DigitalSignal& signal = impl_->signals.at(name);
    signal.value = impl_->sim->getBoolProperty(impl_->sim->handle_scene, mapping.property);
    signal.last_update_ms = static_cast<BusinessTime>(snapshot.generation);
  }
  for (const auto& [name, mapping] : impl_->workcell->tools) {
    ToolState& tool = impl_->tools.at(name);
    const double previous_motor_position =
        impl_->sim->getJointPosition(impl_->tool_motor_joint_handle);
    const double motor_delta = impl_->tool_motor_target - previous_motor_position;
    const double motor_position =
        previous_motor_position + std::clamp(motor_delta, -kToolJointStep, kToolJointStep);
    impl_->sim->setJointPosition(impl_->tool_motor_joint_handle, motor_position);
    const double motor_fraction =
        std::clamp((motor_position - mapping.closed_joint_position) /
                       (mapping.open_joint_position - mapping.closed_joint_position),
                   0.0, 1.0);
    const double aperture = mapping.closed_aperture +
                            motor_fraction * (mapping.open_aperture - mapping.closed_aperture);
    impl_->sim->setJointPosition(impl_->tool_joint_handle, aperture);
    const bool is_open = std::abs(aperture - mapping.open_aperture) <= mapping.aperture_tolerance;
    const bool is_closed =
        std::abs(aperture - mapping.closed_aperture) <= mapping.aperture_tolerance;
    if (is_open) {
      tool.observed = ToolPosition::kOpen;
    } else if (is_closed) {
      tool.observed = ToolPosition::kClosed;
    }
    const bool command_reached = (tool.commanded == ToolPosition::kOpen && is_open) ||
                                 (tool.commanded == ToolPosition::kClosed && is_closed);
    impl_->tool_stable_samples.at(name) =
        command_reached ? impl_->tool_stable_samples.at(name) + 1 : 0;
    tool.last_feedback_ms = static_cast<BusinessTime>(snapshot.generation);
    snapshot.tools.emplace(name, tool);
    snapshot.tool_openings.emplace(name, aperture);
    snapshot.tool_stable_samples.emplace(name, impl_->tool_stable_samples.at(name));
  }
  int64_t parent_handle = impl_->sim->getObjectParent(impl_->workpiece_handle);
  bool attached = parent_handle == impl_->tcp_handle;
  const bool at_pick = IsPoseWithinTolerance(workpiece_pose, pick_pose,
                                             impl_->workcell->pick_radius, 3.141592653589793);
  const bool tcp_in_range = IsPoseWithinTolerance(workpiece_pose, tcp_pose,
                                                  impl_->workcell->pick_radius, 3.141592653589793);
  if (impl_->pending_tool_command.has_value()) {
    const auto& [name, target] = *impl_->pending_tool_command;
    const int stable = impl_->tool_stable_samples.at(name);
    if (stable >= impl_->workcell->stable_steps && target == ToolPosition::kClosed && !attached &&
        !impl_->attachment_requested) {
      if (at_pick && tcp_in_range) {
        impl_->sim->setObjectParent(impl_->workpiece_handle, impl_->tcp_handle, true);
        impl_->attachment_requested = true;
        parent_handle = impl_->sim->getObjectParent(impl_->workpiece_handle);
        attached = parent_handle == impl_->tcp_handle;
      } else {
        snapshot.grip_failed = true;
      }
    }
    if (stable >= impl_->workcell->stable_steps && target == ToolPosition::kOpen && attached &&
        !impl_->release_requested) {
      impl_->sim->setObjectParent(impl_->workpiece_handle, impl_->workcell_root_handle, true);
      impl_->release_requested = true;
      parent_handle = impl_->sim->getObjectParent(impl_->workpiece_handle);
      attached = parent_handle == impl_->tcp_handle;
      workpiece_pose = ToPose(impl_->sim->getObjectPose(impl_->workpiece_handle));
    }
  }
  WorkpieceSnapshot workpiece;
  workpiece.world_pose = workpiece_pose;
  workpiece.at_pick = IsPoseWithinTolerance(workpiece_pose, pick_pose, impl_->workcell->pick_radius,
                                            3.141592653589793);
  workpiece.at_place = IsPoseWithinTolerance(workpiece_pose, place_pose,
                                             impl_->workcell->place_translation_tolerance,
                                             impl_->workcell->place_rotation_tolerance);
  workpiece.attached = attached;
  workpiece.present = workpiece.at_pick || workpiece.attached || workpiece.at_place;
  workpiece.parent_alias = parent_handle == -1 ? "" : impl_->sim->getObjectAlias(parent_handle);
  impl_->attached_stable_samples = workpiece.attached ? impl_->attached_stable_samples + 1 : 0;
  impl_->place_stable_samples =
      !workpiece.attached && workpiece.at_place ? impl_->place_stable_samples + 1 : 0;
  workpiece.attached_stable_samples = impl_->attached_stable_samples;
  workpiece.place_stable_samples = impl_->place_stable_samples;
  if (impl_->pending_tool_command.has_value() &&
      impl_->pending_tool_command->second == ToolPosition::kOpen && impl_->release_requested &&
      !workpiece.at_place) {
    ++impl_->placement_failure_samples;
    snapshot.placement_failed = impl_->placement_failure_samples >= impl_->workcell->stable_steps;
  } else {
    impl_->placement_failure_samples = 0;
  }
  const auto ready = impl_->workcell->signals.find("PART_READY");
  if (ready != impl_->workcell->signals.end()) {
    impl_->signals.at("PART_READY").value = workpiece.present && workpiece.at_pick;
    impl_->sim->setBoolProperty(impl_->sim->handle_scene, ready->second.property,
                                impl_->signals.at("PART_READY").value);
  }
  const auto grip_ok = impl_->workcell->signals.find("GRIP_OK");
  if (grip_ok != impl_->workcell->signals.end()) {
    impl_->signals.at("GRIP_OK").value = workpiece.attached;
    impl_->sim->setBoolProperty(impl_->sim->handle_scene, grip_ok->second.property,
                                impl_->signals.at("GRIP_OK").value);
  }
  snapshot.signals = impl_->signals;
  snapshot.workpiece = std::move(workpiece);
  for (const WorkcellCollisionRule& rule : impl_->workcell->collisions) {
    const int64_t first = impl_->sim->getObject(rule.first_alias);
    const int64_t second = impl_->sim->getObject(rule.second_alias);
    const auto [collides, ignored_pair] = impl_->sim->checkCollision(first, second);
    (void)ignored_pair;
    if (collides != 0 && !rule.permitted) {
      snapshot.collision = true;
      snapshot.collision_aliases.push_back(rule.first_alias + "|" + rule.second_alias);
    }
  }
  return snapshot;
}

WorkcellSnapshot CoppeliaSimBackend::PollWorkcell() {
  if (!impl_->cycle_advanced_by_motion) {
    impl_->sim->step();
  }
  UpdateToolMount();
  impl_->cycle_advanced_by_motion = false;
  return ReadWorkcellSnapshot();
}

WorkcellCommandSubmission CoppeliaSimBackend::RequestSafeState(StopReason reason) {
  (void)reason;
  if (impl_->sim == nullptr || !impl_->workcell.has_value()) {
    return {WorkcellCommandStatus::kRejected, "workcell is not connected", std::nullopt};
  }
  impl_->safe_requested = true;
  try {
    impl_->pending_tool_command.reset();
    impl_->attachment_requested = false;
    impl_->release_requested = false;
    impl_->tool_motor_target = impl_->sim->getJointPosition(impl_->tool_motor_joint_handle);
    for (auto& [name, signal] : impl_->signals) {
      if (signal.direction == SignalDirection::kDo) {
        signal.value = false;
        impl_->sim->setBoolProperty(impl_->sim->handle_scene,
                                    impl_->workcell->signals.at(name).property, false);
      }
    }
  } catch (const std::exception& error) {
    return {WorkcellCommandStatus::kRejected, error.what(), std::nullopt};
  }
  return {WorkcellCommandStatus::kAccepted, "", 0};
}

WorkcellCommandSubmission CoppeliaSimBackend::PollSafeState() {
  if (!impl_->safe_requested || impl_->sim == nullptr || !impl_->workcell.has_value()) {
    return {WorkcellCommandStatus::kRejected, "safe state was not requested", 0};
  }
  try {
    for (const auto& [name, signal] : impl_->signals) {
      if (signal.direction == SignalDirection::kDo &&
          impl_->sim->getBoolProperty(impl_->sim->handle_scene,
                                      impl_->workcell->signals.at(name).property)) {
        return {WorkcellCommandStatus::kRejected, "digital outputs are not safe", 0};
      }
    }
    const double motor_position = impl_->sim->getJointPosition(impl_->tool_motor_joint_handle);
    if (impl_->pending_tool_command.has_value() ||
        std::abs(motor_position - impl_->tool_motor_target) > 1e-6) {
      return {WorkcellCommandStatus::kRejected, "tool actuation has not stopped", 0};
    }
  } catch (const std::exception& error) {
    return {WorkcellCommandStatus::kRejected, error.what(), 0};
  }
  return {WorkcellCommandStatus::kAccepted, "", 0};
}

}  // namespace roborun
