#!/usr/bin/env python3
"""Run and verify the RoboRun GUI evidence capture CoppeliaSim GUI check."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import plistlib
import shlex
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_SCHEMA = "roborun.gui.evidence.v1"
READY_SCHEMA = "roborun.gui.evidence-ready.v1"


class CheckError(RuntimeError):
    def __init__(self, phase: str, message: str, action: str) -> None:
        super().__init__(message)
        self.phase = phase
        self.action = action


def fail(phase: str, message: str, action: str) -> None:
    raise CheckError(phase, message, action)


def quote(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


def print_failure(error: CheckError) -> None:
    print(
        f"RESULT status=failed phase={error.phase} action={quote(error.action)} "
        f"message={quote(str(error))}",
        file=sys.stderr,
    )


def load_json(path: Path, phase: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        fail(phase, f"cannot read valid JSON from {path}: {error}", "Inspect the retained case artifacts.")
    if not isinstance(value, dict):
        fail(phase, f"expected a JSON object in {path}", "Regenerate the case evidence.")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_scalar(raw: str) -> Any:
    if raw == "true":
        return True
    if raw == "false":
        return False
    if raw.startswith("[") and raw.endswith("]"):
        contents = raw[1:-1]
        if not contents:
            return []
        try:
            return [float(item) for item in contents.split(",")]
        except ValueError:
            return raw
    try:
        return int(raw)
    except ValueError:
        pass
    try:
        return float(raw)
    except ValueError:
        return raw


def parse_record(line: str) -> dict[str, Any]:
    tokens = shlex.split(line)
    record: dict[str, Any] = {"record": tokens[0]}
    for token in tokens[1:]:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        record[key] = parse_scalar(value)
    return record


def records(log_text: str, kind: str) -> list[dict[str, Any]]:
    return [parse_record(line) for line in log_text.splitlines() if line.startswith(kind + " ")]


def png_dimensions(path: Path) -> tuple[int, int]:
    try:
        header = path.read_bytes()[:24]
    except OSError as error:
        fail("capture_artifact", f"cannot read screenshot {path}: {error}", "Grant Screen Recording permission and rerun GUI evidence capture.")
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        fail("capture_artifact", f"screenshot is not a valid PNG: {path}", "Use the macOS application-window capture helper.")
    return struct.unpack(">II", header[16:24])


def gif_properties(path: Path) -> tuple[int, int, int]:
    try:
        contents = path.read_bytes()
    except OSError as error:
        fail("animation_artifact", f"cannot read animation {path}: {error}", "Rerun GUI evidence capture window recording.")
    if len(contents) < 14 or contents[:6] not in (b"GIF87a", b"GIF89a"):
        fail("animation_artifact", f"animation is not a GIF: {path}", "Regenerate the GUI evidence capture application-window GIF.")
    width, height = struct.unpack("<HH", contents[6:10])
    cursor = 13
    logical_screen_packed = contents[10]
    if logical_screen_packed & 0x80:
        cursor += 3 * (2 << (logical_screen_packed & 0x07))

    def skip_sub_blocks(position: int) -> int:
        while position < len(contents):
            block_size = contents[position]
            position += 1
            if block_size == 0:
                return position
            position += block_size
            if position > len(contents):
                break
        fail("animation_artifact", f"GIF has a truncated data block: {path}", "Regenerate the GUI evidence capture application-window GIF.")

    frame_count = 0
    found_trailer = False
    while cursor < len(contents):
        marker = contents[cursor]
        cursor += 1
        if marker == 0x3B:
            found_trailer = True
            break
        if marker == 0x21:
            if cursor >= len(contents):
                break
            cursor += 1
            cursor = skip_sub_blocks(cursor)
            continue
        if marker == 0x2C:
            if cursor + 9 > len(contents):
                break
            image_packed = contents[cursor + 8]
            cursor += 9
            if image_packed & 0x80:
                cursor += 3 * (2 << (image_packed & 0x07))
            if cursor >= len(contents):
                break
            cursor += 1
            cursor = skip_sub_blocks(cursor)
            frame_count += 1
            continue
        break
    if not found_trailer or frame_count == 0:
        fail("animation_artifact", f"GIF has no complete image sequence: {path}", "Regenerate the GUI evidence capture application-window GIF.")
    return width, height, frame_count


def require_path(path: Path, phase: str, action: str, executable: bool = False) -> None:
    valid = path.is_file() and (not executable or os.access(path, os.X_OK))
    if not valid:
        kind = "executable" if executable else "file"
        fail(phase, f"required {kind} is missing: {path}", action)


def port_is_occupied(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.2)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def listening_process_ids(port: int) -> set[int]:
    completed = subprocess.run(
        ["/usr/sbin/lsof", "-nP", "-t", f"-iTCP:{port}", "-sTCP:LISTEN"],
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode not in (0, 1):
        fail(
            "port_ownership",
            completed.stderr.strip() or f"cannot inspect the owner of Remote API port {port}",
            "Install the macOS lsof utility and rerun GUI evidence capture.",
        )
    return {int(line) for line in completed.stdout.splitlines() if line.isdigit()}


def preflight(app: Path, python: Path, port: int) -> dict[str, Any]:
    python = Path(os.path.abspath(python))
    binary = app / "Contents" / "MacOS" / "coppeliaSim"
    resources = app / "Contents" / "Resources"
    require_path(
        binary,
        "application",
        "Set --app or COPPELIASIM_APP to a complete CoppeliaSim Edu 4.10 application.",
        executable=True,
    )
    require_path(
        resources / "models" / "robots" / "non-mobile" / "UR5.ttm",
        "application",
        "Reinstall CoppeliaSim Edu 4.10 with its official model resources.",
    )
    require_path(
        resources / "models" / "components" / "grippers" / "RG2.ttm",
        "application",
        "Reinstall CoppeliaSim Edu 4.10 with its official model resources.",
    )
    info_plist = app / "Contents" / "Info.plist"
    require_path(info_plist, "simulator_version", "Reinstall the official CoppeliaSim application bundle.")
    try:
        with info_plist.open("rb") as source:
            version = str(plistlib.load(source).get("CFBundleShortVersionString", ""))
    except (OSError, plistlib.InvalidFileException) as error:
        fail("simulator_version", f"cannot read CoppeliaSim version: {error}", "Reinstall the official application bundle.")
    if not version.startswith("4.10."):
        fail(
            "simulator_version",
            f"GUI evidence capture requires CoppeliaSim Edu 4.10, found {version or 'unreported'}",
            "Install CoppeliaSim Edu 4.10 or select it with --app.",
        )
    require_path(
        python,
        "python_executable",
        "Create a Python environment and pass its interpreter with --python or COPPELIASIM_PYTHON.",
        executable=True,
    )
    modules = subprocess.run(
        [str(python), "-c", "import zmq, cbor2"],
        text=True,
        capture_output=True,
        check=False,
    )
    if modules.returncode != 0:
        fail(
            "python_modules",
            "the configured Python environment is missing pyzmq or cbor2",
            f"Run {python} -m pip install pyzmq cbor2, then rerun GUI evidence capture.",
        )
    if port_is_occupied(port):
        fail(
            "port_occupied",
            f"Remote API port {port} is already owned by another process",
            f"Stop the existing service on port {port}; GUI evidence capture will not connect to or terminate it.",
        )
    if platform.system() != "Darwin" or platform.machine() != "arm64":
        fail(
            "host_platform",
            f"GUI evidence check requires macOS Apple Silicon, found {platform.system()} {platform.machine()}",
            "Run this command on the documented macOS Apple Silicon host.",
        )
    require_path(
        Path("/usr/sbin/lsof"),
        "port_ownership",
        "Restore the macOS lsof utility before running GUI evidence capture.",
        executable=True,
    )
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        check=True,
    ).stdout.strip()
    return {
        "schema": "roborun.gui.environment.v1",
        "source_commit": commit,
        "host": {
            "system": platform.system(),
            "release": platform.mac_ver()[0],
            "machine": platform.machine(),
        },
        "application": str(app.resolve()),
        "simulator_version": version,
        "python": str(python),
        "python_modules": ["zmq", "cbor2"],
        "port": port,
        "launch_mode": "gui",
        "launch_arguments": [
            "-GsimCmd.autoStart=false",
            f"-Gpython={python}",
            f"-GzmqRemoteApi.rpcPort={port}",
        ],
    }


def require_artifact(case_directory: Path, name: str, phase: str) -> Path:
    path = case_directory / name
    require_path(path, phase, "Inspect the retained logs and rerun the failed GUI evidence capture scenario.")
    return path


def load_home_joints() -> list[float]:
    points = load_json(PROJECT_ROOT / "examples" / "config" / "ur5_points.json", "normal_state")
    for point in points.get("points", []):
        if point.get("name") == "HOME":
            return [float(value) for value in point["joints"]]
    fail("normal_state", "HOME point is missing from the accepted CoppeliaSim point catalog", "Restore examples/config/ur5_points.json.")


def validate_lifecycle(log_text: str) -> None:
    lifecycle = records(log_text, "LIFECYCLE")
    passed = {entry.get("operation") for entry in lifecycle if entry.get("status") == "passed"}
    required = {"stop_simulation", "disconnect", "quit_simulator"}
    if not required.issubset(passed):
        fail(
            "cleanup",
            f"missing successful lifecycle operations: {sorted(required - passed)}",
            "Inspect runtime.log and simulator.log; cleanup failures block GUI evidence capture.",
        )


def verify_case(scenario: str, case_directory: Path) -> dict[str, Any]:
    ready_path = require_artifact(case_directory, "ready.json", "evidence_binding")
    runtime_path = require_artifact(case_directory, "runtime.log", "runtime_result")
    simulator_path = require_artifact(case_directory, "simulator.log", "simulator_log")
    environment_path = require_artifact(case_directory, "environment.json", "environment")
    process_path = require_artifact(case_directory, "process.json", "process_result")
    ready = load_json(ready_path, "evidence_binding")
    if ready.get("schema") != READY_SCHEMA or ready.get("scenario") != scenario:
        fail(
            "evidence_binding",
            f"ready record belongs to scenario {ready.get('scenario')!r}, expected {scenario!r}",
            "Use artifacts generated by the same GUI evidence capture scenario run.",
        )
    generation = ready.get("generation")
    if not isinstance(generation, int) or generation <= 0:
        fail("evidence_binding", "ready record has no valid snapshot generation", "Rerun the GUI evidence capture scenario.")
    scenario_id = ready.get("scenario_id")
    if not isinstance(scenario_id, str) or not scenario_id:
        fail("evidence_binding", "ready record has no scenario identifier", "Rerun the GUI evidence capture scenario.")
    screenshot_path = require_artifact(
        case_directory, f"{scenario_id}-generation-{generation}.png", "capture_artifact"
    )
    animation_path = require_artifact(case_directory, f"{scenario_id}.gif", "animation_artifact")
    animation_metadata_path = require_artifact(case_directory, "animation.json", "animation_artifact")
    animation = load_json(animation_metadata_path, "animation_artifact")
    if (
        animation.get("schema") != "roborun.gui.animation.v1"
        or animation.get("scenario") != scenario
        or animation.get("scenario_id") != scenario_id
        or animation.get("file") != animation_path.name
    ):
        fail(
            "evidence_binding",
            "animation metadata does not match the scenario-ready record",
            "Keep GIF, animation metadata and Runtime evidence from the same run.",
        )
    if not isinstance(animation.get("frame_count"), int) or animation["frame_count"] < 2:
        fail("animation_artifact", "GIF has fewer than two recorded frames", "Record the application window throughout execution.")
    if not isinstance(animation.get("frames_per_second"), (int, float)) or animation["frames_per_second"] <= 0:
        fail("animation_artifact", "GIF frame rate is invalid", "Regenerate the application-window GIF.")
    gif_width, gif_height, gif_frame_count = gif_properties(animation_path)
    if gif_frame_count != animation["frame_count"]:
        fail(
            "animation_artifact",
            f"GIF contains {gif_frame_count} frames but metadata records {animation['frame_count']}",
            "Keep GIF and animation metadata from the same encoder run.",
        )
    if animation.get("width") != gif_width or animation.get("height") != gif_height:
        fail(
            "animation_artifact",
            "GIF dimensions do not match animation metadata",
            "Keep GIF and animation metadata from the same encoder run.",
        )
    if gif_width < 640 or gif_height < 480:
        fail(
            "animation_artifact",
            f"application-window GIF is too small: {gif_width}x{gif_height}",
            "Record a visible CoppeliaSim window at the documented size.",
        )
    width, height = png_dimensions(screenshot_path)
    if width < 640 or height < 480:
        fail(
            "capture_artifact",
            f"application-window screenshot is too small: {width}x{height}",
            "Expose and enlarge the CoppeliaSim application window before rerunning GUI evidence capture.",
        )
    environment = load_json(environment_path, "environment")
    if environment.get("launch_mode") != "gui":
        fail("environment", "evidence does not record GUI launch mode", "Run the GUI evidence launcher without headless options.")
    process = load_json(process_path, "process_result")
    expected_exit = 0 if scenario == "normal" else 1
    if process.get("runtime_exit_status") != expected_exit:
        fail(
            "process_result",
            f"runtime exit status is {process.get('runtime_exit_status')}, expected {expected_exit}",
            "Inspect runtime.log for the first failed execution phase.",
        )
    if process.get("cleanup_succeeded") is not True:
        fail("cleanup", "scenario cleanup did not complete", "Stop the owned simulator process and verify the Remote API port is free.")
    log_text = runtime_path.read_text(encoding="utf-8")
    workcells = records(log_text, "WORKCELL")
    results = records(log_text, "RESULT")
    traces = records(log_text, "TRACE")
    if not workcells or not results:
        fail("runtime_result", "runtime log has no structured WORKCELL or RESULT record", "Inspect runtime.log and rerun the scenario.")
    workcell = workcells[-1]
    result = results[-1]
    if workcell.get("generation") != generation:
        fail(
            "evidence_binding",
            f"screenshot generation {generation} does not match Runtime generation {workcell.get('generation')}",
            "Keep screenshot, ready record and runtime log from the same run.",
        )
    joints = workcell.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        fail("runtime_result", "workcell evidence has no six-axis joint snapshot", "Rerun the accepted CoppeliaSim C++ path.")
    pose = workcell.get("pose")
    if not isinstance(pose, list) or len(pose) != 7:
        fail("runtime_result", "workcell evidence has no seven-value current pose", "Rerun the accepted CoppeliaSim C++ path.")
    tool_opening = workcell.get("tool_GRIPPER_opening")
    if not isinstance(tool_opening, (int, float)):
        fail("runtime_result", "workcell evidence has no gripper opening", "Rerun the accepted CoppeliaSim C++ path.")
    final_trace = traces[-1] if traces else {}
    commanded_tool = final_trace.get("commanded_tool")
    observed_tool = final_trace.get("observed_tool")
    if workcell.get("collision") is not False:
        fail("runtime_result", "workcell snapshot reports a collision", "Inspect the frozen scene and Runtime trace.")
    if scenario == "normal":
        if result.get("status") != "passed" or result.get("outcome") != "program_completed":
            fail("normal_state", "normal scenario did not complete", "Inspect runtime.log and rerun the normal scenario.")
        if workcell.get("workpiece_at_place") is not True or workcell.get("workpiece_attached") is not False:
            fail("normal_state", "normal scenario did not leave the released workpiece at PLACE", "Inspect the CoppeliaSim task and frozen workcell snapshot.")
        if commanded_tool != "OPEN" or observed_tool != "OPEN" or float(tool_opening) < 0.05:
            fail("normal_state", "normal scenario did not leave the gripper visibly open", "Inspect the final tool feedback and workcell snapshot.")
        home = load_home_joints()
        if any(abs(float(actual) - expected) > 0.02 for actual, expected in zip(joints, home)):
            fail("normal_state", "normal scenario did not return the robot to HOME", "Inspect the final MOVEJ trace and workcell snapshot.")
    else:
        if result.get("outcome") != "emergency_stopped" or result.get("lifecycle") != "emergency_stopped":
            fail("estop_state", "ESTOP scenario did not reach the emergency-stopped lifecycle", "Inspect ESTOP submission and safe-stop trace ordering.")
        if workcell.get("safe_confirmed") is not True:
            fail("estop_state", "ESTOP workcell safe state was not confirmed", "Inspect Runtime safe-state feedback.")
        estop_indexes = [index for index, entry in enumerate(traces) if entry.get("control") == "estop"]
        if not estop_indexes:
            fail("estop_order", "Trace has no accepted ESTOP control event", "Inject ESTOP while MOVEJ is active.")
        estop_index = estop_indexes[-1]
        active_command_index = traces[estop_index].get("command_index")
        motion_was_active = any(
            entry.get("command_index") == active_command_index
            and entry.get("command") == "MOVEJ"
            and entry.get("command_status") == "running"
            for entry in traces[:estop_index]
        )
        if not motion_was_active:
            fail("estop_order", "ESTOP was not injected during active MOVEJ", "Adjust --estop-after-move-polls to interrupt active motion.")
        if any(
            entry.get("control") == "none"
            and isinstance(entry.get("command_index"), int)
            and entry["command_index"] > active_command_index
            for entry in traces[estop_index + 1 :]
        ):
            fail("estop_order", "a later task command ran after ESTOP", "Keep ESTOP terminal and inspect Runtime control ordering.")
        home = load_home_joints()
        if all(math.isclose(float(actual), expected, abs_tol=0.02) for actual, expected in zip(joints, home)):
            fail("estop_state", "ESTOP pose is indistinguishable from normal HOME completion", "Inject ESTOP earlier during active motion.")
    validate_lifecycle(log_text)
    snapshot = {
        "schema": EVIDENCE_SCHEMA,
        "scenario": scenario,
        "scenario_id": scenario_id,
        "generation": generation,
        "outcome": result.get("outcome"),
        "lifecycle": result.get("lifecycle"),
        "backend": result.get("backend"),
        "backend_version": result.get("backend_version"),
        "joint_positions": joints,
        "current_pose": pose,
        "tool": {
            "name": "GRIPPER",
            "commanded_state": commanded_tool,
            "observed_state": observed_tool,
            "opening": tool_opening,
        },
        "workcell": {
            "scene_schema": workcell.get("scene_schema"),
            "workpiece_present": workcell.get("workpiece_present"),
            "workpiece_attached": workcell.get("workpiece_attached"),
            "workpiece_at_place": workcell.get("workpiece_at_place"),
            "collision": workcell.get("collision"),
            "safe_confirmed": workcell.get("safe_confirmed"),
        },
        "trace": traces,
        "screenshot": {
            "file": screenshot_path.name,
            "format": "png",
            "width": width,
            "height": height,
        },
        "animation": {
            "file": animation_path.name,
            "format": "gif",
            "width": gif_width,
            "height": gif_height,
            "frame_count": gif_frame_count,
            "frames_per_second": animation["frames_per_second"],
        },
    }
    snapshot_path = case_directory / "snapshot.json"
    snapshot_path.write_text(json.dumps(snapshot, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    artifact_paths = {
        "screenshot": screenshot_path,
        "animation": animation_path,
        "animation_metadata": animation_metadata_path,
        "snapshot": snapshot_path,
        "runtime_log": runtime_path,
        "simulator_log": simulator_path,
        "environment": environment_path,
        "process": process_path,
        "ready": ready_path,
    }
    manifest = {
        "schema": EVIDENCE_SCHEMA,
        "scenario": scenario,
        "scenario_id": scenario_id,
        "generation": generation,
        "source_commit": environment.get("source_commit"),
        "artifacts": {
            name: {"path": path.name, "sha256": sha256(path), "bytes": path.stat().st_size}
            for name, path in artifact_paths.items()
        },
    }
    (case_directory / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return manifest


def run_logged(command: list[str], log_path: Path, phase: str, action: str) -> None:
    with log_path.open("a", encoding="utf-8") as output:
        output.write("COMMAND " + shlex.join(command) + "\n")
        output.flush()
        completed = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            stdout=output,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    if completed.returncode != 0:
        fail(phase, f"command exited with status {completed.returncode}: {shlex.join(command)}", action)


def build_runner(build_directory: Path, environment: dict[str, Any]) -> tuple[Path, Path]:
    build_directory.mkdir(parents=True, exist_ok=True)
    build_log = build_directory / "build.log"
    build_log.unlink(missing_ok=True)
    resources = Path(environment["application"]) / "Contents" / "Resources"
    run_logged(
        [
            "cmake",
            "-S",
            str(PROJECT_ROOT),
            "-B",
            str(build_directory),
            "-DBUILD_TESTING=ON",
            "-DROBORUN_ENABLE_COPPELIASIM=ON",
            f"-DCOPPELIASIM_ROOT_DIR={resources}",
        ],
        build_log,
        "build_configure",
        "Inspect build.log and the configured CoppeliaSim Resources directory.",
    )
    run_logged(
        ["cmake", "--build", str(build_directory), "--target", "roborun", "--parallel", "2"],
        build_log,
        "build_compile",
        "Inspect build.log and fix the first compiler error.",
    )
    capture_helper = build_directory / "macos_coppeliasim_window"
    run_logged(
        [
            "swiftc",
            str(PROJECT_ROOT / "scripts" / "macos_coppeliasim_window.swift"),
            "-o",
            str(capture_helper),
        ],
        build_log,
        "capture_helper_build",
        "Install the macOS command-line developer tools and rerun GUI evidence capture.",
    )
    return build_directory / "roborun", capture_helper


def wait_for_port(port: int, simulator: subprocess.Popen[Any], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if simulator.poll() is not None:
            fail(
                "gui_early_exit",
                f"CoppeliaSim GUI exited with status {simulator.returncode} before Remote API readiness",
                "Inspect simulator.log for the first startup error.",
            )
        if port_is_occupied(port):
            owners = listening_process_ids(port)
            if simulator.pid in owners:
                return
            if owners:
                fail(
                    "port_owner_mismatch",
                    f"Remote API port {port} is owned by PID(s) {sorted(owners)}, not the launched CoppeliaSim PID {simulator.pid}",
                    "Stop the unrelated listener; GUI evidence capture will only connect to the simulator process it launched.",
                )
        time.sleep(0.2)
    fail(
        "remote_api_readiness",
        f"CoppeliaSim did not open Remote API port {port} within {timeout:g} seconds",
        "Inspect simulator.log and verify the ZMQ Remote API add-on is enabled.",
    )


def wait_for_file(path: Path, process: subprocess.Popen[Any], timeout: float, phase: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.is_file():
            return
        if process.poll() is not None:
            fail(
                phase,
                f"Runtime exited with status {process.returncode} before publishing {path.name}",
                "Inspect runtime.log for the first failed Runtime or lifecycle record.",
            )
        time.sleep(0.05)
    fail(
        phase,
        f"Runtime did not publish {path.name} within {timeout:g} seconds",
        "Inspect runtime.log and simulator.log; evidence waits are intentionally bounded.",
    )


def confirm_capture(ready_path: Path, continue_path: Path) -> None:
    temporary_path = continue_path.with_suffix(continue_path.suffix + ".tmp")
    temporary_path.write_bytes(ready_path.read_bytes())
    os.replace(temporary_path, continue_path)


def locate_window(
    capture_helper: Path,
    simulator: subprocess.Popen[Any],
    timeout: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    last_error = "target window was not visible"
    window: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        if simulator.poll() is not None:
            fail(
                "capture_window",
                "CoppeliaSim GUI exited before window capture",
                "Inspect simulator.log and rerun the scenario.",
            )
        located = subprocess.run(
            [str(capture_helper), "locate", str(simulator.pid)],
            text=True,
            capture_output=True,
            check=False,
            timeout=max(1.0, min(5.0, deadline - time.monotonic())),
        )
        if located.returncode == 2:
            fail(
                "capture_permission",
                located.stderr.strip() or "Screen Recording permission is unavailable",
                "Grant Screen Recording permission to the terminal host in System Settings, then rerun GUI evidence capture.",
            )
        if located.returncode == 0:
            try:
                window = json.loads(located.stdout)
                break
            except json.JSONDecodeError:
                last_error = "window helper returned invalid metadata"
        else:
            last_error = located.stderr.strip() or last_error
        time.sleep(0.2)
    if window is None:
        fail(
            "capture_window",
            f"CoppeliaSim application window was not available within {timeout:g} seconds: {last_error}",
            "Keep the owned CoppeliaSim window visible on the logged-in desktop.",
        )
    return window


def capture_window_id(window_id: int, screenshot_path: Path, timeout: float) -> None:
    screenshot_path.unlink(missing_ok=True)
    try:
        captured = subprocess.run(
            [
                "/usr/sbin/screencapture",
                "-x",
                "-o",
                "-t",
                "png",
                "-l",
                str(window_id),
                str(screenshot_path),
            ],
            text=True,
            capture_output=True,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        fail(
            "capture_timeout",
            f"application-window capture exceeded {timeout:g} seconds",
            "Check Screen Recording permission and keep the CoppeliaSim window visible.",
        )
    if captured.returncode != 0 or not screenshot_path.is_file():
        fail(
            "capture_permission",
            captured.stderr.strip() or "macOS did not create an application-window screenshot",
            "Grant Screen Recording permission to the terminal host in System Settings, then rerun GUI evidence capture.",
        )
    width, height = png_dimensions(screenshot_path)
    if width < 640 or height < 480:
        fail(
            "capture_artifact",
            f"captured CoppeliaSim window is too small: {width}x{height}",
            "Expose and enlarge the CoppeliaSim window before rerunning GUI evidence capture.",
        )


class WindowRecorder:
    def __init__(
        self,
        window: dict[str, Any],
        frame_directory: Path,
        frames_per_second: float,
        capture_timeout: float,
    ) -> None:
        if frames_per_second <= 0:
            fail("animation_configuration", "GIF frame rate must be positive", "Pass a positive --gif-fps value.")
        self.window = window
        self.frame_directory = frame_directory
        self.frames_per_second = frames_per_second
        self.capture_timeout = capture_timeout
        self.frames: list[Path] = []
        self.error: CheckError | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _capture_next_frame(self) -> None:
        frame_path = self.frame_directory / f"frame-{len(self.frames):05d}.png"
        capture_window_id(int(self.window["window_id"]), frame_path, self.capture_timeout)
        self.frames.append(frame_path)

    def _record(self) -> None:
        interval = 1.0 / self.frames_per_second
        next_frame_at = time.monotonic()
        while not self._stop.is_set():
            try:
                self._capture_next_frame()
            except CheckError as error:
                self.error = error
                return
            next_frame_at += interval
            self._stop.wait(max(0.0, next_frame_at - time.monotonic()))

    def start(self) -> None:
        self.frame_directory.mkdir(parents=True, exist_ok=False)
        self._capture_next_frame()
        self._thread = threading.Thread(target=self._record, name="gui_evidence-window-recorder", daemon=True)
        self._thread.start()

    def stop(self) -> list[Path]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(5.0, self.capture_timeout + 1.0))
            if self._thread.is_alive():
                fail(
                    "animation_capture",
                    "application-window recorder did not stop within its bounded timeout",
                    "Inspect Screen Recording permission and the retained frame directory.",
                )
        if self.error is not None:
            raise self.error
        return list(self.frames)


def create_animation(
    capture_helper: Path,
    scenario: str,
    scenario_id: str,
    case_directory: Path,
    window: dict[str, Any],
    frame_paths: list[Path],
    final_screenshot: Path,
    frames_per_second: float,
    timeout: float,
) -> dict[str, Any]:
    animation_path = case_directory / f"{scenario_id}.gif"
    all_frames = [*frame_paths, final_screenshot]
    if len(all_frames) < 2:
        fail(
            "animation_capture",
            "application-window recording produced fewer than two frames",
            "Keep CoppeliaSim visible and rerun the GUI evidence capture scenario.",
        )
    animation_path.unlink(missing_ok=True)
    try:
        generated = subprocess.run(
            [
                str(capture_helper),
                "gif",
                str(animation_path),
                f"{frames_per_second:g}",
                *(str(path) for path in all_frames),
            ],
            text=True,
            capture_output=True,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        fail(
            "animation_encode",
            f"GIF encoding exceeded {timeout:g} seconds",
            "Inspect the retained PNG frames and rerun with a lower --gif-fps value.",
        )
    if generated.returncode != 0 or not animation_path.is_file():
        fail(
            "animation_encode",
            generated.stderr.strip() or "GIF encoder did not create an animation",
            "Inspect the retained PNG frames and rerun GUI evidence capture.",
        )
    try:
        encoder_metadata = json.loads(generated.stdout)
    except json.JSONDecodeError:
        fail("animation_encode", "GIF encoder returned invalid metadata", "Inspect build.log and rerun GUI evidence capture.")
    try:
        inspected = subprocess.run(
            [str(capture_helper), "inspect", str(animation_path)],
            text=True,
            capture_output=True,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        fail("animation_encode", "GIF decode check timed out", "Inspect the retained PNG frames and rerun GUI evidence capture.")
    if inspected.returncode != 0:
        fail(
            "animation_encode",
            inspected.stderr.strip() or "ImageIO could not decode the generated GIF",
            "Inspect the retained PNG frames and rerun GUI evidence capture.",
        )
    try:
        decoded_metadata = json.loads(inspected.stdout)
    except json.JSONDecodeError:
        fail("animation_encode", "GIF decode check returned invalid metadata", "Inspect build.log and rerun GUI evidence capture.")
    if decoded_metadata != {
        "frame_count": encoder_metadata.get("frame_count"),
        "height": encoder_metadata.get("height"),
        "width": encoder_metadata.get("width"),
    }:
        fail(
            "animation_encode",
            "GIF decoded frame count or dimensions differ from encoder metadata",
            "Inspect the retained PNG frames and rerun GUI evidence capture.",
        )
    metadata = {
        "schema": "roborun.gui.animation.v1",
        "scenario": scenario,
        "scenario_id": scenario_id,
        "file": animation_path.name,
        "frame_count": encoder_metadata.get("frame_count"),
        "frames_per_second": encoder_metadata.get("frames_per_second"),
        "width": encoder_metadata.get("width"),
        "height": encoder_metadata.get("height"),
        "window_id": window.get("window_id"),
        "window_title": window.get("title"),
    }
    (case_directory / "animation.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metadata


def stop_owned_process(process: subprocess.Popen[Any] | None, timeout: float = 10.0) -> bool:
    if process is None or process.poll() is not None:
        return True
    process.terminate()
    try:
        process.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        process.kill()
        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False


def wait_for_port_release(port: int, timeout: float = 15.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not port_is_occupied(port):
            return True
        time.sleep(0.1)
    return False


def scenario_identifier(scenario: str) -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{scenario}-{timestamp}-{uuid.uuid4().hex[:8]}"


def write_failure_record(
    case_directory: Path,
    error: CheckError,
    process_record: dict[str, Any],
) -> None:
    failure_record = {
        "schema": "roborun.gui.failure.v1",
        "phase": error.phase,
        "message": str(error),
        "action": error.action,
        "failed_at": datetime.now(timezone.utc).isoformat(),
        "process": process_record,
        "available_artifacts": sorted(
            path.name for path in case_directory.iterdir() if path.is_file() and path.name != "failure.json"
        ),
    }
    (case_directory / "failure.json").write_text(
        json.dumps(failure_record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def run_scenario(
    scenario: str,
    check_directory: Path,
    environment: dict[str, Any],
    roborun_binary: Path,
    capture_helper: Path,
    startup_timeout: float,
    scenario_timeout: float,
    capture_timeout: float,
    evidence_timeout: float,
    gif_frames_per_second: float,
) -> dict[str, Any]:
    port = int(environment["port"])
    if port_is_occupied(port):
        fail(
            "port_occupied",
            f"Remote API port {port} is occupied before {scenario}",
            "Stop the unrelated service; GUI evidence capture will not connect to or terminate it.",
        )
    case_directory = check_directory / scenario
    case_directory.mkdir(parents=True, exist_ok=False)
    scenario_id = scenario_identifier(scenario)
    ready_path = case_directory / "ready.json"
    continue_path = case_directory / "continue.json"
    runtime_log_path = case_directory / "runtime.log"
    simulator_log_path = case_directory / "simulator.log"
    process_path = case_directory / "process.json"
    scene_path = case_directory / "workcell.ttt"
    application = Path(environment["application"])
    simulator_binary = application / "Contents" / "MacOS" / "coppeliaSim"
    resources = application / "Contents" / "Resources"
    launch_arguments = [str(simulator_binary), *environment["launch_arguments"]]
    case_environment = {
        **environment,
        "scenario": scenario,
        "scenario_id": scenario_id,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "launch_command": launch_arguments,
    }
    (case_directory / "environment.json").write_text(
        json.dumps(case_environment, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    simulator: subprocess.Popen[Any] | None = None
    runtime: subprocess.Popen[Any] | None = None
    runtime_status: int | None = None
    capture_error: CheckError | None = None
    scenario_error: CheckError | None = None
    recorder: WindowRecorder | None = None
    cleanup_succeeded = True
    simulator_output = simulator_log_path.open("w", encoding="utf-8")
    runtime_output = runtime_log_path.open("w", encoding="utf-8")
    try:
        simulator_output.write("COMMAND " + shlex.join(launch_arguments) + "\n")
        simulator_output.flush()
        simulator = subprocess.Popen(
            launch_arguments,
            cwd=PROJECT_ROOT,
            stdin=subprocess.DEVNULL,
            stdout=simulator_output,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        case_environment["simulator_pid"] = simulator.pid
        (case_directory / "environment.json").write_text(
            json.dumps(case_environment, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        wait_for_port(port, simulator, startup_timeout)
        window = locate_window(capture_helper, simulator, capture_timeout)
        case_environment["captured_window"] = window
        recorder = WindowRecorder(
            window,
            case_directory / "animation-frames",
            gif_frames_per_second,
            capture_timeout,
        )
        recorder.start()
        command = [
            str(roborun_binary),
            "run",
            "--backend",
            "coppeliasim",
            "--coppeliasim-port",
            str(port),
            "--robot-config",
            str(PROJECT_ROOT / "examples" / "config" / "ur5_robot.json"),
            "--points-config",
            str(PROJECT_ROOT / "examples" / "config" / "ur5_points.json"),
            "--io-config",
            str(PROJECT_ROOT / "examples" / "config" / "workcell_io.json"),
            "--tool-config",
            str(PROJECT_ROOT / "examples" / "config" / "tool_feedback.json"),
            "--workcell-config",
            str(PROJECT_ROOT / "examples" / "config" / "workcell.json"),
            "--resources",
            str(resources),
            "--scene",
            str(scene_path),
            "--program",
            str(PROJECT_ROOT / "examples" / "tasks" / "pick_place.task"),
            "--format",
            "trace",
            "--evidence-ready",
            str(ready_path),
            "--evidence-continue",
            str(continue_path),
            "--evidence-scenario",
            scenario,
            "--evidence-scenario-id",
            scenario_id,
            "--evidence-timeout-ms",
            str(int(evidence_timeout * 1000)),
        ]
        if scenario == "estop":
            command.extend(["--estop-after-move-polls", "20"])
        runtime_output.write("COMMAND " + shlex.join(command) + "\n")
        runtime_output.flush()
        runtime = subprocess.Popen(
            command,
            cwd=PROJECT_ROOT,
            stdout=runtime_output,
            stderr=subprocess.STDOUT,
            text=True,
        )
        wait_for_file(ready_path, runtime, scenario_timeout, "evidence_ready")
        ready = load_json(ready_path, "evidence_binding")
        if ready.get("scenario") != scenario or ready.get("scenario_id") != scenario_id:
            fail(
                "evidence_binding",
                "Runtime evidence-ready record does not match this scenario",
                "Keep each case in a fresh evidence directory.",
            )
        screenshot_path = case_directory / f"{scenario_id}-generation-{ready.get('generation')}.png"
        try:
            frame_paths = recorder.stop()
            capture_window_id(int(window["window_id"]), screenshot_path, capture_timeout)
            screenshot_width, screenshot_height = png_dimensions(screenshot_path)
            window["captured_width"] = screenshot_width
            window["captured_height"] = screenshot_height
            create_animation(
                capture_helper,
                scenario,
                scenario_id,
                case_directory,
                window,
                frame_paths,
                screenshot_path,
                gif_frames_per_second,
                max(30.0, scenario_timeout),
            )
            for frame_path in frame_paths:
                frame_path.unlink(missing_ok=True)
            recorder.frame_directory.rmdir()
        except CheckError as error:
            capture_error = error
        finally:
            confirm_capture(ready_path, continue_path)
        try:
            runtime_status = runtime.wait(timeout=30)
        except subprocess.TimeoutExpired:
            capture_error = capture_error or CheckError(
                "runtime_exit", "Runtime did not exit after capture confirmation", "Inspect runtime.log and evidence barrier cleanup."
            )
        if simulator.poll() is None:
            try:
                simulator.wait(timeout=30)
            except subprocess.TimeoutExpired:
                capture_error = capture_error or CheckError(
                    "gui_exit", "CoppeliaSim did not exit after quitSimulator", "Inspect lifecycle records and simulator.log."
                )
        case_environment["finished_at"] = datetime.now(timezone.utc).isoformat()
        (case_directory / "environment.json").write_text(
            json.dumps(case_environment, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    except CheckError as error:
        scenario_error = error
        raise
    finally:
        if recorder is not None:
            try:
                recorder.stop()
            except CheckError as error:
                capture_error = capture_error or error
        cleanup_succeeded = stop_owned_process(runtime) and cleanup_succeeded
        if runtime is not None and runtime_status is None:
            runtime_status = runtime.poll()
        cleanup_succeeded = stop_owned_process(simulator) and cleanup_succeeded
        cleanup_succeeded = wait_for_port_release(port) and cleanup_succeeded
        runtime_output.close()
        simulator_output.close()
        process_record = {
            "runtime_exit_status": runtime_status,
            "simulator_exit_status": simulator.poll() if simulator is not None else None,
            "cleanup_succeeded": cleanup_succeeded,
            "port_released": not port_is_occupied(port),
        }
        process_path.write_text(
            json.dumps(process_record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        retained_error = scenario_error or capture_error
        if retained_error is not None:
            write_failure_record(case_directory, retained_error, process_record)
    simulator_text = simulator_log_path.read_text(encoding="utf-8", errors="replace")
    if "No module named 'zmq'" in simulator_text or "No module named 'cbor2'" in simulator_text:
        module_error = CheckError(
            "python_modules",
            "CoppeliaSim reported a missing pyzmq or cbor2 module after preflight",
            "Pass the same configured interpreter with --python and inspect simulator.log.",
        )
        write_failure_record(case_directory, module_error, process_record)
        raise module_error
    if capture_error is not None:
        raise capture_error
    try:
        return verify_case(scenario, case_directory)
    except CheckError as error:
        write_failure_record(case_directory, error, process_record)
        raise


def run_check(arguments: argparse.Namespace) -> Path:
    environment = preflight(arguments.app, arguments.python, arguments.port)
    build_directory = arguments.build_dir.resolve()
    roborun_binary, capture_helper = build_runner(build_directory, environment)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + environment["source_commit"][:8]
    check_directory = build_directory / "evidence" / run_id
    check_directory.mkdir(parents=True)
    manifests: dict[str, Any] = {}
    scenarios = ("normal", "estop") if arguments.scenario == "all" else (arguments.scenario,)
    for scenario in scenarios:
        manifests[scenario] = run_scenario(
            scenario,
            check_directory,
            environment,
            roborun_binary,
            capture_helper,
            arguments.startup_timeout,
            arguments.scenario_timeout,
            arguments.capture_timeout,
            arguments.evidence_timeout,
            arguments.gif_fps,
        )
    combined = {
        "schema": EVIDENCE_SCHEMA,
        "run_id": run_id,
        "source_commit": environment["source_commit"],
        "scenario_order": list(scenarios),
        "cases": {
            scenario: {
                "manifest": f"{scenario}/manifest.json",
                "sha256": sha256(check_directory / scenario / "manifest.json"),
            }
            for scenario in scenarios
        },
        "visual_review": "required",
    }
    (check_directory / "manifest.json").write_text(
        json.dumps(combined, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return check_directory


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    preflight_parser = subparsers.add_parser("preflight", help="validate the GUI host without launching CoppeliaSim")
    preflight_parser.add_argument("--app", type=Path, required=True)
    preflight_parser.add_argument("--python", type=Path, required=True)
    preflight_parser.add_argument("--port", type=int, default=23000)
    verify_parser = subparsers.add_parser("verify", help="verify one retained scenario evidence directory")
    verify_parser.add_argument("--scenario", choices=("normal", "estop"), required=True)
    verify_parser.add_argument("--case-dir", type=Path, required=True)
    run_parser = subparsers.add_parser("run", help="run the real CoppeliaSim GUI check")
    run_parser.add_argument(
        "--app",
        type=Path,
        default=Path(os.environ.get("COPPELIASIM_APP", "")),
        required="COPPELIASIM_APP" not in os.environ,
    )
    run_parser.add_argument(
        "--python",
        type=Path,
        default=Path(os.environ.get("COPPELIASIM_PYTHON", "")),
        required="COPPELIASIM_PYTHON" not in os.environ,
    )
    run_parser.add_argument("--build-dir", type=Path, default=PROJECT_ROOT / ".build" / "gui-evidence")
    run_parser.add_argument("--scenario", choices=("all", "normal", "estop"), default="all")
    run_parser.add_argument("--port", type=int, default=23000)
    run_parser.add_argument("--startup-timeout", type=float, default=60.0)
    run_parser.add_argument("--scenario-timeout", type=float, default=120.0)
    run_parser.add_argument("--capture-timeout", type=float, default=20.0)
    run_parser.add_argument("--evidence-timeout", type=float, default=60.0)
    run_parser.add_argument("--gif-fps", type=float, default=4.0)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        if arguments.command == "preflight":
            environment = preflight(arguments.app, arguments.python, arguments.port)
            print(json.dumps(environment, sort_keys=True))
            print("RESULT status=passed phase=gui_preflight")
        elif arguments.command == "verify":
            manifest = verify_case(arguments.scenario, arguments.case_dir)
            print(
                f"RESULT status=passed phase=gui_case_verification scenario={arguments.scenario} "
                f"generation={manifest['generation']}"
            )
        elif arguments.command == "run":
            check_directory = run_check(arguments)
            print(
                "RESULT status=passed phase=gui_machine_check "
                f"scenario={arguments.scenario} evidence={quote(str(check_directory))}"
            )
        return 0
    except CheckError as error:
        print_failure(error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
