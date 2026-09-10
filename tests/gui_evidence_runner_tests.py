#!/usr/bin/env python3
"""Behavior tests for the public GUI evidence runner commands."""

from __future__ import annotations

import json
import importlib.util
from pathlib import Path
import plistlib
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
import zlib


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNNER = PROJECT_ROOT / "scripts" / "gui_evidence_capture.py"


def load_runner_module():
    specification = importlib.util.spec_from_file_location("gui_evidence_capture", RUNNER)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {RUNNER}")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def write_png(path: Path, width: int = 900, height: int = 700) -> None:
    def chunk(kind: bytes, data: bytes) -> bytes:
        payload = kind + data
        return struct.pack(">I", len(data)) + payload + struct.pack(">I", zlib.crc32(payload))

    row = b"\x00" + b"\x80\x90\xa0" * width
    pixels = zlib.compress(row * height)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", pixels)
        + chunk(b"IEND", b"")
    )


def write_gif_header(path: Path, width: int = 900, height: int = 700) -> None:
    logical_screen = struct.pack("<HHBBB", width, height, 0x80, 0, 0)
    color_table = b"\x00\x00\x00\xff\xff\xff"
    frame = (
        b"\x21\xf9\x04\x00\x01\x00\x00\x00"
        b"\x2c\x00\x00\x00\x00\x01\x00\x01\x00\x00"
        b"\x02\x02\x44\x01\x00"
    )
    path.write_bytes(b"GIF89a" + logical_screen + color_table + frame + frame + b"\x3b")


class GuiEvidenceRunnerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def run_runner(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(RUNNER), *arguments],
            cwd=PROJECT_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )

    def make_app(self, version: str = "4.10.0 rev0") -> Path:
        app = self.root / "coppeliaSim.app"
        binary = app / "Contents" / "MacOS" / "coppeliaSim"
        resources = app / "Contents" / "Resources"
        binary.parent.mkdir(parents=True)
        binary.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        binary.chmod(0o755)
        for model in (
            resources / "models" / "robots" / "non-mobile" / "UR5.ttm",
            resources / "models" / "components" / "grippers" / "RG2.ttm",
        ):
            model.parent.mkdir(parents=True, exist_ok=True)
            model.write_text("fixture", encoding="utf-8")
        with (app / "Contents" / "Info.plist").open("wb") as output:
            plistlib.dump({"CFBundleShortVersionString": version}, output)
        return app

    def make_python(self, modules_available: bool = True) -> Path:
        executable = self.root / "coppeliasim-python"
        status = 0 if modules_available else 1
        executable.write_text(f"#!/bin/sh\nexit {status}\n", encoding="utf-8")
        executable.chmod(0o755)
        return executable

    def test_preflight_reports_missing_application_with_corrective_action(self) -> None:
        result = self.run_runner(
            "preflight",
            "--app",
            str(self.root / "missing.app"),
            "--python",
            str(self.make_python()),
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=application", result.stderr)
        self.assertIn("action=", result.stderr)

    def test_preflight_rejects_incomplete_python_before_launch(self) -> None:
        result = self.run_runner(
            "preflight",
            "--app",
            str(self.make_app()),
            "--python",
            str(self.make_python(modules_available=False)),
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=python_modules", result.stderr)
        self.assertIn("pyzmq", result.stderr)
        self.assertIn("cbor2", result.stderr)

    def test_preflight_does_not_disturb_an_unrelated_port_owner(self) -> None:
        owner = socket.socket()
        owner.bind(("127.0.0.1", 0))
        owner.listen()
        port = owner.getsockname()[1]
        try:
            result = self.run_runner(
                "preflight",
                "--app",
                str(self.make_app()),
                "--python",
                str(self.make_python()),
                "--port",
                str(port),
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("phase=port_occupied", result.stderr)
            self.assertEqual(owner.getsockname()[1], port)
        finally:
            owner.close()

    def make_case(self, scenario: str, generation: int = 42) -> Path:
        case_directory = self.root / scenario
        case_directory.mkdir()
        scenario_id = f"{scenario}-run-123"
        screenshot = case_directory / f"{scenario_id}-generation-{generation}.png"
        write_png(screenshot)
        animation = case_directory / f"{scenario_id}.gif"
        write_gif_header(animation)
        (case_directory / "animation.json").write_text(
            json.dumps(
                {
                    "schema": "roborun.gui.animation.v1",
                    "scenario": scenario,
                    "scenario_id": scenario_id,
                    "file": animation.name,
                    "frame_count": 2,
                    "frames_per_second": 4,
                    "width": 900,
                    "height": 700,
                }
            ),
            encoding="utf-8",
        )
        ready = {
            "schema": "roborun.gui.evidence-ready.v1",
            "scenario": scenario,
            "scenario_id": scenario_id,
            "generation": generation,
        }
        (case_directory / "ready.json").write_text(json.dumps(ready), encoding="utf-8")
        (case_directory / "environment.json").write_text(
            json.dumps({"source_commit": "0123456789abcdef", "launch_mode": "gui"}),
            encoding="utf-8",
        )
        expected_exit = 0 if scenario == "normal" else 1
        (case_directory / "process.json").write_text(
            json.dumps({"runtime_exit_status": expected_exit, "cleanup_succeeded": True}),
            encoding="utf-8",
        )
        if scenario == "normal":
            trace = (
                'TRACE time_ms=440 command_index=8 command=MOVEJ control=none '
                'outcome=none actual=[0,-0.35,0.3,-0.25,0.15,-0.1]\n'
                'TRACE time_ms=440 command_index=9 command=STOP control=none outcome=program_completed '
                'commanded_tool=OPEN observed_tool=OPEN\n'
                f'WORKCELL generation={generation} scene_schema="roborun.workcell.v1" '
                'workpiece_present=true workpiece_attached=false workpiece_at_place=true '
                'collision=false safe_confirmed=false simulation_running=true '
                'joints=[0,-0.35,0.3,-0.25,0.15,-0.1] pose=[0.1,0.2,0.3,0,0,0,1] '
                'tool_GRIPPER_opening=0.085\n'
                'RESULT status=passed backend=coppeliasim backend_version="4.10.0 rev0" '
                'final_state=stopped lifecycle=stopped outcome=program_completed\n'
            )
        else:
            trace = (
                'TRACE time_ms=20 command_index=4 command=MOVEJ command_status=running control=none '
                'outcome=none actual=[0.05,-0.39,0.34,-0.29,0.17,-0.12]\n'
                'TRACE time_ms=20 command_index=4 command=UNKNOWN command_status=pending control=estop '
                'control_source="cli-check" outcome=emergency_stopped\n'
                'TRACE time_ms=20 command_index=4 command=MOVEJ command_status=cancelled control=none '
                'outcome=none actual=[0.05,-0.39,0.34,-0.29,0.17,-0.12] '
                'commanded_tool=OPEN observed_tool=OPEN\n'
                f'WORKCELL generation={generation} scene_schema="roborun.workcell.v1" '
                'workpiece_present=true workpiece_attached=false workpiece_at_place=false '
                'collision=false safe_confirmed=true simulation_running=true '
                'joints=[0.05,-0.39,0.34,-0.29,0.17,-0.12] pose=[0.1,0.2,0.3,0,0,0,1] '
                'tool_GRIPPER_opening=0.085\n'
                'RESULT status=failed backend=coppeliasim backend_version="4.10.0 rev0" '
                'final_state=stopped lifecycle=emergency_stopped outcome=emergency_stopped\n'
            )
        (case_directory / "runtime.log").write_text(
            trace
            + "LIFECYCLE operation=stop_simulation status=passed\n"
            + "LIFECYCLE operation=disconnect status=passed\n"
            + "LIFECYCLE operation=quit_simulator status=passed\n",
            encoding="utf-8",
        )
        (case_directory / "simulator.log").write_text(
            "CoppeliaSim v4.10.0 (rev. 0), macOS\n", encoding="utf-8"
        )
        return case_directory

    def test_verify_binds_normal_snapshot_screenshot_trace_and_hashes(self) -> None:
        case_directory = self.make_case("normal")

        result = self.run_runner(
            "verify", "--scenario", "normal", "--case-dir", str(case_directory)
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        snapshot = json.loads((case_directory / "snapshot.json").read_text(encoding="utf-8"))
        manifest = json.loads((case_directory / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(snapshot["generation"], 42)
        self.assertTrue(snapshot["workcell"]["workpiece_at_place"])
        self.assertEqual(manifest["scenario"], "normal")
        self.assertEqual(manifest["scenario_id"], "normal-run-123")
        self.assertIn("screenshot", manifest["artifacts"])
        self.assertIn("animation", manifest["artifacts"])
        self.assertEqual(len(manifest["artifacts"]["screenshot"]["sha256"]), 64)
        self.assertEqual(snapshot["animation"]["frame_count"], 2)
        self.assertEqual(snapshot["current_pose"], [0.1, 0.2, 0.3, 0, 0, 0, 1])
        self.assertEqual(snapshot["tool"]["observed_state"], "OPEN")

    def test_verify_accepts_estop_only_when_motion_was_active_and_no_later_command_ran(self) -> None:
        case_directory = self.make_case("estop")

        result = self.run_runner(
            "verify", "--scenario", "estop", "--case-dir", str(case_directory)
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        snapshot = json.loads((case_directory / "snapshot.json").read_text(encoding="utf-8"))
        self.assertEqual(snapshot["outcome"], "emergency_stopped")
        self.assertTrue(snapshot["workcell"]["safe_confirmed"])

    def test_verify_rejects_a_screenshot_bound_to_another_scenario(self) -> None:
        case_directory = self.make_case("estop")

        result = self.run_runner(
            "verify", "--scenario", "normal", "--case-dir", str(case_directory)
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=evidence_binding", result.stderr)

    def test_verify_rejects_a_later_task_command_after_estop(self) -> None:
        case_directory = self.make_case("estop")
        runtime_path = case_directory / "runtime.log"
        runtime = runtime_path.read_text(encoding="utf-8")
        runtime = runtime.replace(
            "WORKCELL generation=42",
            "TRACE time_ms=20 command_index=5 command=SET_TOOL command_status=accepted control=none\n"
            "WORKCELL generation=42",
        )
        runtime_path.write_text(runtime, encoding="utf-8")

        result = self.run_runner(
            "verify", "--scenario", "estop", "--case-dir", str(case_directory)
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=estop_order", result.stderr)

    def test_verify_rejects_a_truncated_gif_with_claimed_frames(self) -> None:
        case_directory = self.make_case("normal")
        animation = next(case_directory.glob("*.gif"))
        animation.write_bytes(b"GIF89a" + struct.pack("<HH", 900, 700) + b"fixture")

        result = self.run_runner(
            "verify", "--scenario", "normal", "--case-dir", str(case_directory)
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=animation_artifact", result.stderr)


class GuiCaptureFailureTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.runner = load_runner_module()

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def make_executable(self, path: Path, contents: str) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")
        path.chmod(0o755)
        return path

    def unused_port(self) -> int:
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        return port

    def run_capture_failure(self, helper_status: int) -> tuple[Exception, Path]:
        port = self.unused_port()
        application = self.root / f"status-{helper_status}" / "coppeliaSim.app"
        self.make_executable(
            application / "Contents" / "MacOS" / "coppeliaSim",
            "#!/usr/bin/env python3\n"
            "import socket, time\n"
            "server = socket.socket()\n"
            "server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)\n"
            f"server.bind(('127.0.0.1', {port}))\n"
            "server.listen()\n"
            "time.sleep(30)\n",
        )
        helper = self.make_executable(
            self.root / f"capture-helper-{helper_status}",
            "#!/bin/sh\n"
            f"echo 'fixture capture failure' >&2\nexit {helper_status}\n",
        )
        runtime = self.make_executable(self.root / "unused-runtime", "#!/bin/sh\nexit 99\n")
        check = self.root / f"evidence-{helper_status}"
        check.mkdir()
        environment = {
            "application": str(application),
            "launch_arguments": [],
            "port": port,
            "source_commit": "0123456789abcdef",
            "launch_mode": "gui",
        }

        with self.assertRaises(self.runner.CheckError) as raised:
            self.runner.run_scenario(
                "normal",
                check,
                environment,
                runtime,
                helper,
                2.0,
                2.0,
                0.05,
                2.0,
                4.0,
            )

        case_directory = check / "normal"
        process = json.loads((case_directory / "process.json").read_text(encoding="utf-8"))
        self.assertTrue(process["cleanup_succeeded"])
        self.assertTrue(process["port_released"])
        self.assertFalse(self.runner.port_is_occupied(port))
        self.assertTrue((case_directory / "simulator.log").is_file())
        failure = json.loads((case_directory / "failure.json").read_text(encoding="utf-8"))
        self.assertEqual(failure["phase"], raised.exception.phase)
        self.assertTrue(failure["process"]["cleanup_succeeded"])
        return raised.exception, case_directory

    def test_screen_recording_permission_failure_cleans_owned_process_and_port(self) -> None:
        error, _ = self.run_capture_failure(2)

        self.assertEqual(error.phase, "capture_permission")

    def test_missing_owned_window_cleans_owned_process_and_port(self) -> None:
        error, _ = self.run_capture_failure(3)

        self.assertEqual(error.phase, "capture_window")

    def test_screenshot_timeout_has_a_stable_failure_phase(self) -> None:
        screenshot = self.root / "timed-out.png"
        with mock.patch.object(
            self.runner.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired(["screencapture"], timeout=0.01),
        ):
            with self.assertRaises(self.runner.CheckError) as raised:
                self.runner.capture_window_id(123, screenshot, 0.01)

        self.assertEqual(raised.exception.phase, "capture_timeout")
        self.assertFalse(screenshot.exists())

    def test_recording_thread_propagates_capture_failure(self) -> None:
        recorder = self.runner.WindowRecorder(
            {"window_id": 123}, self.root / "frames", 20.0, 0.1
        )
        capture_count = 0

        def capture_then_fail(*_arguments) -> None:
            nonlocal capture_count
            capture_count += 1
            if capture_count > 1:
                raise self.runner.CheckError(
                    "animation_capture", "fixture recorder failure", "inspect fixture"
                )

        with mock.patch.object(self.runner, "capture_window_id", side_effect=capture_then_fail):
            recorder.start()
            deadline = time.monotonic() + 1.0
            while recorder.error is None and time.monotonic() < deadline:
                time.sleep(0.01)
            with self.assertRaises(self.runner.CheckError) as raised:
                recorder.stop()

        self.assertEqual(raised.exception.phase, "animation_capture")

    def test_readiness_rejects_an_unrelated_listener_after_launch(self) -> None:
        owner = socket.socket()
        owner.bind(("127.0.0.1", 0))
        owner.listen()
        port = owner.getsockname()[1]
        simulator = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            with self.assertRaises(self.runner.CheckError) as raised:
                self.runner.wait_for_port(port, simulator, 1.0)
        finally:
            simulator.terminate()
            simulator.wait(timeout=2)
            owner.close()

        self.assertEqual(raised.exception.phase, "port_owner_mismatch")


if __name__ == "__main__":
    unittest.main()
