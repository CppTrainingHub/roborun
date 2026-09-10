# RoboRun

RoboRun 是一个 C++20 机器人任务运行时。它读取任务文件和机器人配置，完成静态校验，再根据设备反馈执行运动、数字 I/O、工具动作和等待。运行中支持暂停、恢复、停止、急停与复位，并提供状态快照、执行记录和报警。

核心库可独立使用。Mock 后端用于本地运行和测试；CoppeliaSim 后端提供 UR5、RG2 夹爪和工件搬运仿真；可选 ROS 2 Bridge 提供 Action、Service 和 Topic 接口。

## 构建与测试

需要 CMake 3.24+ 和支持 C++20 的编译器。支持 macOS 和 Linux；ROS 2 示例使用 Ubuntu 22.04 / Humble。JSON 与 GoogleTest 源码已随仓库提供，核心构建无需下载依赖。

```bash
git clone https://github.com/CppTrainingHub/roborun.git
cd roborun
cmake -S . -B .build/core -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build .build/core --parallel 4
ctest --test-dir .build/core --output-on-failure --no-tests=error
```

## 运行任务

```bash
.build/core/roborun run --backend mock --program examples/tasks/ur5_move.task
```

任务示例：

```text
SERVO_ON
MOVEJ 0.20 -0.35 0.30 -0.25 0.15 -0.10
STOP
```

关节角使用弧度，STOP 必须是任务的最后一条指令。命名点位需要加载机器人和点位配置：

```bash
.build/core/roborun run --backend mock --format trace \
  --robot-config examples/config/ur5_robot.json \
  --points-config examples/config/ur5_points.json \
  --program examples/tasks/ur5_named_points.task
```

`MOVEJ PICK SPEED 50` 使用配置中参考关节速度的 50%。其他指令包括 `SERVO_OFF`、`DELAY`、`SET_DO`、`WAIT_DI` 和 `SET_TOOL`。包含 I/O、夹爪与延迟反馈的示例：

```bash
.build/core/roborun run --backend mock --format trace \
  --robot-config examples/config/ur5_robot.json \
  --points-config examples/config/ur5_points.json \
  --io-config examples/config/digital_io.json \
  --tool-config examples/config/tool_feedback.json \
  --scenario examples/config/mock_workcell.scenario.json \
  --program examples/tasks/mock_workcell.task
```

默认输出适合直接阅读，`--format trace` 输出结构化执行记录。`accepted` 表示设备接受命令，`succeeded` 表示命令完成。正常完成返回退出码 0；无效输入、运行故障、操作员停止和急停返回非零，具体原因见 `outcome` 和诊断字段。

## 作为 C++ 库使用

```bash
cmake --install .build/core --prefix "$PWD/.build/install"
cmake -S tests/consumer -B .build/consumer \
  -DCMAKE_PREFIX_PATH="$PWD/.build/install"
cmake --build .build/consumer --parallel 4
ctest --test-dir .build/consumer --output-on-failure --no-tests=error
```

下游 CMake 使用 `find_package(RoboRun 1.0 CONFIG REQUIRED)`，链接 `RoboRun::core`。`RuntimeSession` 支持逐步执行和控制请求；`RobotTaskRuntime::Execute()` 提供一次性执行入口。Session 和 Backend 由同一个线程驱动，跨线程请求应由宿主排队转交。

## CoppeliaSim

需要单独安装 CoppeliaSim 4.10，包含官方 UR5、RG2 模型和 ZeroMQ Remote API C++ 客户端。模型与模拟器不随本仓库分发。核心构建默认关闭仿真适配器。

编译适配器还需要 Git、Python 3 和 ZeroMQ 开发库。macOS 可通过 Homebrew 安装 `zeromq`，Ubuntu 对应 `libzmq3-dev`。官方客户端在配置时会从 GitHub 获取 jsoncons 和 cppzmq，因此首次构建需要网络。

macOS 无界面运行示例：

```bash
python3 -m venv .venv/coppeliasim
.venv/coppeliasim/bin/python -m pip install pyzmq cbor2
export COPPELIASIM_APP="$HOME/Applications/coppeliaSim.app"
export COPPELIASIM_PYTHON="$PWD/.venv/coppeliasim/bin/python"
export ROBORUN_BUILD_DIR="$PWD/.build/coppeliasim"
ROBORUN_SINGLE_RUN=1 scripts/run_coppeliasim_mvp.sh
```

完整搬运及故障示例运行 `scripts/run_coppeliasim_workcell.sh`，使用相同环境变量。请关闭占用 23000 端口的其他服务；脚本会创建独立仿真进程并在运行后清理。运行任务会加载模型或重建场景，请勿连接含未保存工作的模拟器实例。

手动构建适配器时，将 `COPPELIASIM_ROOT_DIR` 指向含 `programming/` 的目录：macOS 为应用包内的 `Contents/Resources`，Linux 为安装目录。

```bash
cmake -S . -B .build/coppeliasim \
  -DROBORUN_ENABLE_COPPELIASIM=ON \
  -DCOPPELIASIM_ROOT_DIR=/path/to/CoppeliaSim/Resources
cmake --build .build/coppeliasim --parallel 4
```

GUI 采集工具为 `scripts/gui_evidence_capture.py` 和 `scripts/linux_gui_evidence.py`，参数可通过 `--help` 查看。它们将窗口画面、运行状态和日志保存到 `.build/`；macOS 需要屏幕录制权限，Linux GUI 容器通过 Xvfb/noVNC 提供画面。

## ROS 2

参见 [ROS 2 Bridge 使用说明](ros2/README.md)。两个 ROS 包直接复用仓库根目录的核心库，构建时请保留整个仓库结构。

## 范围与依赖

RoboRun 面向任务编排和仿真集成，不提供真实电机控制、硬实时保证或硬件安全认证。应用层 ESTOP 需要设备反馈确认，不能替代实体急停回路。仿真抓取使用场景对象关系表达附着，不等同于完整接触动力学。

第三方组件及许可证见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
