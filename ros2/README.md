# RoboRun ROS 2 Bridge

`roborun_interfaces` 定义 Action、Service 和状态消息，`roborun_ros2_bridge` 通过一个工作线程驱动 Runtime。核心 CMake 构建无需 ROS 2。

## 构建与启动

在 Ubuntu 22.04 / ROS 2 Humble 中，从 RoboRun 仓库根目录执行以下命令。保留整个仓库结构，两个 ROS 包会通过相对路径构建根目录的核心库。

```bash
source /opt/ros/humble/setup.bash
colcon --log-base .build/ros2/log build --base-paths ros2 \
  --packages-select roborun_interfaces roborun_ros2_bridge \
  --build-base .build/ros2/build --install-base .build/ros2/install
source .build/ros2/install/setup.bash
ros2 launch roborun_ros2_bridge mock_bridge.launch.py
```

Docker 用户可直接运行 `scripts/run_ros2_mock_docker.sh`，脚本构建 Humble 镜像并运行接口测试和 DDS 客户端。

## 提交和控制

在另一个终端加载相同 ROS 环境后，提交任务：

```bash
source /opt/ros/humble/setup.bash
source .build/ros2/install/setup.bash
ros2 action send_goal /execute_program roborun_interfaces/action/ExecuteProgram \
  "{program_text: \"SERVO_ON\\nMOVEJ 0 0 0 0 0 0\\nSTOP\\n\", source_name: demo.task}" --feedback
ros2 service call /control_runtime roborun_interfaces/srv/ControlRuntime \
  "{control: ESTOP, source: operator}"
ros2 service call /control_runtime roborun_interfaces/srv/ControlRuntime \
  "{control: RESET, source: operator}"
```

Action Goal 包含 `program_text` 和 `source_name`，上限分别为 64 KiB 和 256 bytes。配置在节点启动时通过 `robot_config`、`points_config`、`io_config`、`tool_config`、`scenario_config` 和 `workcell_config` 参数指定。

取消某个 Goal 时，Action 客户端使用该 Goal handle 的 `cancel_goal_async()`；控制整个运行时则调用 `/control_runtime`，control 可为 PAUSE、RESUME、STOP、ESTOP 或 RESET。Service 返回 queued 只表示请求入队，执行效果需继续观察状态。Action Cancel 等待停止确认后返回 canceled；停止确认失败会返回 aborted。

任务解析失败时，Action 的 outcome 为 rejected，生命周期和快照序列保持 Runtime 当前真实状态。急停锁存不会被无效任务解除。

## 状态消息

`/joint_states` 使用 best-effort、keep-last(5)，六个关节位置来自同一次快照，未知的 velocity 和 effort 保持为空。`/runtime_status`、`/io_state`、`/alarm_state` 使用 reliable、transient-local、keep-last(1)，在 DDS 发现和 QoS 匹配后，晚加入的订阅者可读取最近状态。

自定义状态消息携带 `snapshot_sequence` 与 `business_time_ms`。标准 JointState 使用 header stamp，不含 snapshot_sequence；各 Topic 的时间戳来自同一次发布。生命周期和 outcome 使用小写字符串，例如 `running`、`emergency_stopped`；诊断码使用 `MOTION_TIMEOUT` 等大写字符串。

## CoppeliaSim 后端

在相同构建命令后追加：

```bash
--cmake-args -DROBORUN_BRIDGE_ENABLE_COPPELIASIM=ON \
  -DCOPPELIASIM_ROOT_DIR=/path/to/CoppeliaSim/Resources
```

启动节点时提供 `backend=coppeliasim`，以及 `model_path`、`resources_path`、`scene_path` 和相应工作站配置。macOS 宿主连接 Humble 容器的完整运行示例见 `scripts/run_ros2_coppeliasim_docker.sh`。

当前 Bridge 在任务终态后断开后端连接。CoppeliaSim 工作站急停后，RESET 所需的同会话现场复查可能无法完成；此时重新启动节点和场景，不要把 RESET 当作自动重连命令。
