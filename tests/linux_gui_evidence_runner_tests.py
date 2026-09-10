#!/usr/bin/env python3
"""Behavior tests for the public Linux GUI evidence check commands."""

from __future__ import annotations

import json
import importlib.util
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import zlib


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNNER = PROJECT_ROOT / "scripts" / "linux_gui_evidence.py"


def load_runner_module():
    specification = importlib.util.spec_from_file_location("linux_gui_evidence", RUNNER)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {RUNNER}")
    module = importlib.util.module_from_spec(specification)
    sys.path.insert(0, str(RUNNER.parent))
    try:
        specification.loader.exec_module(module)
    finally:
        sys.path.remove(str(RUNNER.parent))
    return module


def write_png(path: Path, width: int = 900, height: int = 700) -> None:
    def chunk(kind: bytes, data: bytes) -> bytes:
        payload = kind + data
        return struct.pack(">I", len(data)) + payload + struct.pack(">I", zlib.crc32(payload))

    row = b"\x00" + b"\x80\x90\xa0" * width
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(row * height))
        + chunk(b"IEND", b"")
    )


def write_gif(path: Path, width: int = 900, height: int = 700) -> None:
    logical_screen = struct.pack("<HHBBB", width, height, 0x80, 0, 0)
    color_table = b"\x00\x00\x00\xff\xff\xff"
    frame = (
        b"\x21\xf9\x04\x00\x01\x00\x00\x00"
        b"\x2c\x00\x00\x00\x00\x01\x00\x01\x00\x00"
        b"\x02\x02\x44\x01\x00"
    )
    path.write_bytes(b"GIF89a" + logical_screen + color_table + frame + frame + b"\x3b")


class LinuxGuiEvidenceRunnerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.docker_settings = self.root / "docker-settings.json"
        self.docker_settings.write_text(
            json.dumps({"UseVirtualizationFrameworkRosetta": True}), encoding="utf-8"
        )

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def run_runner(
        self, *arguments: str, environment: dict[str, str] | None = None
    ) -> subprocess.CompletedProcess[str]:
        execution_environment = dict(os.environ)
        if environment is not None:
            execution_environment.update(environment)
        execution_environment.setdefault(
            "ROBORUN_DOCKER_SETTINGS", str(self.docker_settings)
        )
        return subprocess.run(
            [sys.executable, str(RUNNER), *arguments],
            cwd=PROJECT_ROOT,
            text=True,
            capture_output=True,
            check=False,
            env=execution_environment,
        )

    def make_fake_docker(
        self,
        *,
        build_status: int = 0,
        image_architecture: str = "amd64",
        run_status: int = 0,
    ) -> tuple[Path, Path]:
        command_log = self.root / "docker-commands.jsonl"
        executable = self.root / "docker"
        executable.write_text(
            "#!/usr/bin/env python3\n"
            "import json, os, pathlib, sys, time\n"
            f"log = pathlib.Path({str(command_log)!r})\n"
            "with log.open('a', encoding='utf-8') as output:\n"
            "    output.write(json.dumps(sys.argv[1:]) + '\\n')\n"
            "args = sys.argv[1:]\n"
            "if args[:1] == ['version']:\n"
            "    time.sleep(float(os.environ.get('FAKE_DOCKER_VERSION_SECONDS', '0')))\n"
            "    print(json.dumps({'Client': {'Version': '29.6.1', 'Os': 'darwin', 'Arch': 'arm64'}, "
            "'Server': {'Version': '29.6.1', 'Platform': {'Name': 'Docker Desktop 4.81.0'}, "
            "'Os': 'linux', 'Arch': 'arm64'}}))\n"
            "elif args[:1] == ['info']:\n"
            "    print(json.dumps({'OperatingSystem': 'Docker Desktop', 'Architecture': 'aarch64'}))\n"
            "elif args[:2] == ['buildx', 'build']:\n"
            "    time.sleep(float(os.environ.get('FAKE_DOCKER_BUILD_SECONDS', '0')))\n"
            f"    sys.exit({build_status})\n"
            "elif args[:2] == ['image', 'inspect']:\n"
            f"    print(json.dumps([{{'Id': 'sha256:fixture', 'Architecture': {image_architecture!r}, "
            "'Os': 'linux', 'RepoDigests': []}]))\n"
            "elif args[:1] == ['run']:\n"
            "    time.sleep(float(os.environ.get('FAKE_DOCKER_RUN_SECONDS', '0')))\n"
            f"    sys.exit({run_status})\n"
            "elif args[:1] in (['stop'], ['rm']):\n"
            "    pass\n"
            "else:\n"
            "    print('unsupported fake docker command: ' + repr(args), file=sys.stderr)\n"
            "    sys.exit(97)\n",
            encoding="utf-8",
        )
        executable.chmod(0o755)
        return executable, command_log

    def test_default_host_readiness_budget_covers_container_source_builds(self) -> None:
        runner = load_runner_module()
        arguments = runner.build_parser().parse_args(["run"])

        self.assertGreaterEqual(
            arguments.novnc_timeout,
            arguments.source_build_timeout * 2 + 120,
        )

    def test_preflight_records_docker_host_target_and_rosetta_decision(self) -> None:
        docker, _ = self.make_fake_docker()

        result = self.run_runner("preflight", "--docker", str(docker))

        self.assertEqual(result.returncode, 0, result.stderr)
        environment = json.loads(result.stdout.splitlines()[0])
        self.assertEqual(environment["schema"], "roborun.linux-gui.environment.v1")
        self.assertEqual(environment["container"]["platform"], "linux/amd64")
        self.assertEqual(environment["compatibility"]["mode"], "rosetta")
        self.assertIn("Docker Desktop", environment["docker"]["server_platform"])

    def test_preflight_requires_explicit_mode_when_rosetta_setting_is_unknown(self) -> None:
        docker, _ = self.make_fake_docker()
        unknown_settings = self.root / "unknown-settings.json"
        unknown_settings.write_text("{}", encoding="utf-8")

        result = self.run_runner(
            "preflight",
            "--docker",
            str(docker),
            environment={"ROBORUN_DOCKER_SETTINGS": str(unknown_settings)},
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=compatibility_detection", result.stderr)

    def test_preflight_bounds_a_hung_docker_query(self) -> None:
        docker, _ = self.make_fake_docker()

        result = self.run_runner(
            "preflight",
            "--docker",
            str(docker),
            "--docker-timeout",
            "0.01",
            environment={"FAKE_DOCKER_VERSION_SECONDS": "0.2"},
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=docker_unavailable", result.stderr)
        self.assertIn("exceeded", result.stderr)

    def test_preflight_reports_missing_docker_with_corrective_action(self) -> None:
        result = self.run_runner(
            "preflight", "--docker", str(self.root / "missing-docker")
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=docker_unavailable", result.stderr)
        self.assertIn("action=", result.stderr)

    def test_preflight_rejects_an_occupied_published_port_without_disturbing_owner(self) -> None:
        docker, _ = self.make_fake_docker()
        owner = socket.socket()
        owner.bind(("127.0.0.1", 0))
        owner.listen()
        port = owner.getsockname()[1]
        try:
            result = self.run_runner(
                "preflight", "--docker", str(docker), "--novnc-port", str(port)
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("phase=port_occupied", result.stderr)
            self.assertEqual(owner.getsockname()[1], port)
        finally:
            owner.close()

    def test_run_reports_image_build_failure_at_its_actual_boundary(self) -> None:
        docker, _ = self.make_fake_docker(build_status=19)

        result = self.run_runner(
            "run",
            "--docker",
            str(docker),
            "--build-dir",
            str(self.root / "build"),
            "--scenario",
            "normal",
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=image_build", result.stderr)
        self.assertIn("Dockerfile", result.stderr)

    def test_run_bounds_a_hung_image_build(self) -> None:
        docker, _ = self.make_fake_docker()

        result = self.run_runner(
            "run",
            "--docker",
            str(docker),
            "--build-dir",
            str(self.root / "build-timeout"),
            "--scenario",
            "normal",
            "--image-build-timeout",
            "0.01",
            environment={"FAKE_DOCKER_BUILD_SECONDS": "0.2"},
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=image_build_timeout", result.stderr)

    def test_run_rejects_a_non_amd64_image_before_container_launch(self) -> None:
        docker, command_log = self.make_fake_docker(image_architecture="arm64")

        result = self.run_runner(
            "run",
            "--docker",
            str(docker),
            "--build-dir",
            str(self.root / "build"),
            "--scenario",
            "normal",
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=image_architecture", result.stderr)
        commands = command_log.read_text(encoding="utf-8")
        self.assertNotIn('["run"', commands)

    def make_case(self, scenario: str = "normal", generation: int = 42) -> Path:
        case_directory = self.root / scenario
        case_directory.mkdir()
        scenario_id = f"{scenario}-linux-run-123"
        screenshot = case_directory / f"{scenario_id}-generation-{generation}.png"
        animation = case_directory / f"{scenario_id}.gif"
        write_png(screenshot)
        write_gif(animation)
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
        (case_directory / "ready.json").write_text(
            json.dumps(
                {
                    "schema": "roborun.gui.evidence-ready.v1",
                    "scenario": scenario,
                    "scenario_id": scenario_id,
                    "generation": generation,
                }
            ),
            encoding="utf-8",
        )
        environment = {
            "schema": "roborun.linux-gui.environment.v1",
            "source_commit": "0123456789abcdef",
            "launch_mode": "gui",
            "host": {"system": "Darwin", "machine": "arm64"},
            "container": {"os": "linux", "architecture": "amd64", "platform": "linux/amd64"},
            "docker": {
                "image_id": "sha256:fixture",
                "server_version": "29.6.1",
                "server_platform": "Docker Desktop 4.81.0",
            },
            "simulator": {
                "version": "4.10.0 rev0",
                "archive_url": "https://downloads.coppeliarobotics.com/fixture.tar.xz",
                "archive_sha256": "a" * 64,
            },
            "display": {
                "xvfb": True,
                "window_manager": "fluxbox",
                "renderer": "llvmpipe",
                "managed_window": True,
            },
            "captured_window": {
                "owner_pid": 123,
                "frame_window_id": 456,
                "title": "CoppeliaSim",
                "capture_width": 900,
                "capture_height": 700,
            },
            "novnc": {"reachable_from_host": True, "url": "http://127.0.0.1:6080/vnc.html"},
            "compatibility": {
                "mode": "rosetta",
                "simSubprocess": "disabled",
                "reason": "darwin_arm64_docker_desktop_linux_amd64",
            },
        }
        (case_directory / "environment.json").write_text(json.dumps(environment), encoding="utf-8")
        (case_directory / "process.json").write_text(
            json.dumps(
                {
                    "runtime_exit_status": 0,
                    "simulator_exit_status": 0,
                    "xvfb_exit_status": 0,
                    "window_manager_exit_status": 0,
                    "x11vnc_exit_status": 0,
                    "novnc_exit_status": 0,
                    "cleanup_succeeded": True,
                    "container_removed": True,
                    "ports_released": {"remote_api": True, "vnc": True, "novnc": True},
                }
            ),
            encoding="utf-8",
        )
        (case_directory / "runtime.log").write_text(
            'TRACE time_ms=440 command_index=9 command=STOP control=none outcome=program_completed '
            'commanded_tool=OPEN observed_tool=OPEN\n'
            f'WORKCELL generation={generation} scene_schema="roborun.workcell.v1" '
            'workpiece_present=true workpiece_attached=false workpiece_at_place=true '
            'collision=false safe_confirmed=false simulation_running=true '
            'joints=[0,-0.35,0.3,-0.25,0.15,-0.1] pose=[0.1,0.2,0.3,0,0,0,1] '
            'tool_GRIPPER_opening=0.085\n'
            'RESULT status=passed backend=coppeliasim backend_version="4.10.0 rev0" '
            'final_state=stopped lifecycle=stopped outcome=program_completed\n'
            'LIFECYCLE operation=stop_simulation status=passed\n'
            'LIFECYCLE operation=disconnect status=passed\n'
            'LIFECYCLE operation=quit_simulator status=passed\n',
            encoding="utf-8",
        )
        (case_directory / "simulator.log").write_text(
            "CoppeliaSim v4.10.0 (rev. 0), Linux\n", encoding="utf-8"
        )
        (case_directory / "container.log").write_text("fixture container log\n", encoding="utf-8")
        return case_directory

    def test_verify_binds_linux_provenance_novnc_and_cleanup_to_gui_state(self) -> None:
        case_directory = self.make_case()

        result = self.run_runner(
            "verify", "--scenario", "normal", "--case-dir", str(case_directory)
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        manifest = json.loads((case_directory / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["schema"], "roborun.linux-gui.evidence.v1")
        self.assertEqual(manifest["environment"]["image_id"], "sha256:fixture")
        self.assertIn("container_log", manifest["artifacts"])

    def test_verify_rejects_a_case_without_host_reachable_novnc(self) -> None:
        case_directory = self.make_case()
        environment_path = case_directory / "environment.json"
        environment = json.loads(environment_path.read_text(encoding="utf-8"))
        environment["novnc"]["reachable_from_host"] = False
        environment_path.write_text(json.dumps(environment), encoding="utf-8")

        result = self.run_runner(
            "verify", "--scenario", "normal", "--case-dir", str(case_directory)
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=novnc_evidence", result.stderr)

    def test_verify_rejects_incomplete_display_and_window_evidence(self) -> None:
        case_directory = self.make_case()
        environment_path = case_directory / "environment.json"
        environment = json.loads(environment_path.read_text(encoding="utf-8"))
        environment["display"]["renderer"] = ""
        environment_path.write_text(json.dumps(environment), encoding="utf-8")
        result = self.run_runner("verify", "--scenario", "normal", "--case-dir", str(case_directory))
        self.assertIn("phase=display_evidence", result.stderr)

        environment["display"]["renderer"] = "llvmpipe"
        environment["captured_window"]["frame_window_id"] = None
        environment_path.write_text(json.dumps(environment), encoding="utf-8")
        result = self.run_runner("verify", "--scenario", "normal", "--case-dir", str(case_directory))
        self.assertIn("phase=window_evidence", result.stderr)

    def test_verify_requires_exit_status_for_every_owned_gui_service(self) -> None:
        case_directory = self.make_case()
        process_path = case_directory / "process.json"
        process = json.loads(process_path.read_text(encoding="utf-8"))
        process["novnc_exit_status"] = None
        process_path.write_text(json.dumps(process), encoding="utf-8")

        result = self.run_runner(
            "verify", "--scenario", "normal", "--case-dir", str(case_directory)
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=process_evidence", result.stderr)

    def test_verify_rejects_capture_animation_barrier_runtime_and_cleanup_failures(self) -> None:
        screenshot_case = self.make_case()
        next(screenshot_case.glob("*.png")).write_bytes(b"not a png")
        result = self.run_runner("verify", "--scenario", "normal", "--case-dir", str(screenshot_case))
        self.assertIn("phase=capture_artifact", result.stderr)

        self.root = self.root / "gif"
        self.root.mkdir()
        gif_case = self.make_case()
        next(gif_case.glob("*.gif")).write_bytes(b"GIF89a truncated")
        result = self.run_runner("verify", "--scenario", "normal", "--case-dir", str(gif_case))
        self.assertIn("phase=animation_artifact", result.stderr)

        self.root = self.root / "barrier"
        self.root.mkdir()
        barrier_case = self.make_case()
        (barrier_case / "ready.json").unlink()
        result = self.run_runner("verify", "--scenario", "normal", "--case-dir", str(barrier_case))
        self.assertIn("phase=evidence_binding", result.stderr)

        self.root = self.root / "runtime"
        self.root.mkdir()
        runtime_case = self.make_case()
        process_path = runtime_case / "process.json"
        process = json.loads(process_path.read_text(encoding="utf-8"))
        process["runtime_exit_status"] = 9
        process_path.write_text(json.dumps(process), encoding="utf-8")
        result = self.run_runner("verify", "--scenario", "normal", "--case-dir", str(runtime_case))
        self.assertIn("phase=process_result", result.stderr)

        self.root = self.root / "cleanup"
        self.root.mkdir()
        cleanup_case = self.make_case()
        process_path = cleanup_case / "process.json"
        process = json.loads(process_path.read_text(encoding="utf-8"))
        process["container_removed"] = False
        process_path.write_text(json.dumps(process), encoding="utf-8")
        result = self.run_runner("verify", "--scenario", "normal", "--case-dir", str(cleanup_case))
        self.assertIn("phase=cleanup", result.stderr)

    def test_window_manager_readiness_rejects_missing_x11_atom(self) -> None:
        runner = load_runner_module()
        completed = subprocess.CompletedProcess(
            ["xprop"], 0, stdout="_NET_SUPPORTING_WM_CHECK:  no such atom on any window.\n", stderr=""
        )
        with mock.patch.object(runner.subprocess, "run", return_value=completed):
            self.assertFalse(runner.window_manager_is_ready())

    def test_window_manager_readiness_accepts_fluxbox_window_property(self) -> None:
        runner = load_runner_module()
        completed = subprocess.CompletedProcess(
            ["xprop"],
            0,
            stdout=(
                "_NET_SUPPORTING_WM_CHECK(WINDOW): window id # 0x200022\n"
            ),
            stderr="",
        )
        with mock.patch.object(runner.subprocess, "run", return_value=completed):
            self.assertTrue(runner.window_manager_is_ready(0.5))

    def test_remote_api_readiness_requires_a_bounded_protocol_response(self) -> None:
        runner = load_runner_module()
        simulator_root = self.root / "CoppeliaSim"
        client_root = (
            simulator_root
            / "programming"
            / "zmqRemoteApi"
            / "clients"
            / "python"
            / "src"
        )
        client_root.mkdir(parents=True)

        completed = subprocess.CompletedProcess(["python3"], 0, stdout="", stderr="")
        with mock.patch.object(
            runner.subprocess, "run", return_value=completed
        ) as run:
            self.assertTrue(
                runner.remote_api_is_ready(simulator_root, 23000, 0.5)
            )
        command = run.call_args.args[0]
        self.assertIn(str(client_root), command)
        self.assertIn("getSimulationState", " ".join(command))
        self.assertEqual(run.call_args.kwargs["timeout"], 0.5)

        unavailable = subprocess.CompletedProcess(
            ["python3"], 1, stdout="", stderr="not CoppeliaSim"
        )
        with mock.patch.object(runner.subprocess, "run", return_value=unavailable):
            self.assertFalse(
                runner.remote_api_is_ready(simulator_root, 23000, 0.5)
            )

        with mock.patch.object(
            runner.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired(["python3", "-c"], 0.01),
        ):
            with self.assertRaises(runner.CheckError) as context:
                runner.remote_api_is_ready(simulator_root, 23000, 0.01)
        self.assertEqual(context.exception.phase, "remote_api_readiness")

    def test_renderer_probe_timeout_reports_the_display_boundary(self) -> None:
        runner = load_runner_module()
        with mock.patch.object(
            runner.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired(["glxinfo", "-B"], 0.01),
        ):
            with self.assertRaises(runner.CheckError) as context:
                runner.renderer_name(0.01)
        self.assertEqual(context.exception.phase, "opengl")
        self.assertIn("exceeded", str(context.exception))

    def test_display_probe_and_renderer_reject_unavailable_services(self) -> None:
        runner = load_runner_module()
        unavailable = subprocess.CompletedProcess(
            ["xdpyinfo"], 1, stdout="", stderr="cannot open display"
        )
        with mock.patch.object(runner.subprocess, "run", return_value=unavailable):
            self.assertFalse(runner.display_is_ready(":99", 0.5))

        invalid_renderer = subprocess.CompletedProcess(
            ["glxinfo", "-B"], 0, stdout="OpenGL vendor string: Mesa\n", stderr=""
        )
        with mock.patch.object(
            runner.subprocess, "run", return_value=invalid_renderer
        ):
            with self.assertRaises(runner.CheckError) as context:
                runner.renderer_name(0.5)
        self.assertEqual(context.exception.phase, "opengl")

    def test_window_discovery_filters_external_dialog_and_small_windows(self) -> None:
        runner = load_runner_module()
        simulator = mock.Mock(pid=42)
        simulator.poll.return_value = None

        def probe(command, *_args, **_kwargs):
            if command[1:3] == ["search", "--onlyvisible"]:
                return subprocess.CompletedProcess(
                    command, 0, stdout="100\n101\n102\n103\n", stderr=""
                )
            if command[1] == "getwindowpid":
                owner = "99" if command[2] == "100" else "42"
                return subprocess.CompletedProcess(
                    command, 0, stdout=owner + "\n", stderr=""
                )
            if command[0] == "xprop":
                kind = (
                    "_NET_WM_WINDOW_TYPE_DIALOG"
                    if command[2] == "101"
                    else "_NET_WM_WINDOW_TYPE_NORMAL"
                )
                return subprocess.CompletedProcess(
                    command, 0, stdout=kind + "\n", stderr=""
                )
            if command[1] == "getwindowgeometry":
                size = "WIDTH=200\nHEIGHT=100\n" if command[-1] == "102" else "WIDTH=900\nHEIGHT=700\nX=10\nY=20\n"
                return subprocess.CompletedProcess(
                    command, 0, stdout=size, stderr=""
                )
            if command[1] == "getwindowname":
                return subprocess.CompletedProcess(
                    command, 0, stdout="CoppeliaSim\n", stderr=""
                )
            raise AssertionError(command)

        with (
            mock.patch.object(runner, "owned_process_ids", return_value={42}),
            mock.patch.object(runner, "run_bounded_probe", side_effect=probe),
            mock.patch.object(runner, "frame_extents", return_value=(1, 1, 22, 3)),
            mock.patch.object(runner, "frame_window_id", return_value=88),
        ):
            window = runner.locate_linux_window(simulator, 0.5, 0.1)

        self.assertEqual(window["window_id"], 103)
        self.assertEqual(window["owner_pid"], 42)
        self.assertEqual(window["frame_window_id"], 88)

    def test_window_discovery_reports_early_exit_and_no_owned_window(self) -> None:
        runner = load_runner_module()
        exited = mock.Mock(pid=42, returncode=7)
        exited.poll.return_value = 7
        with self.assertRaises(runner.CheckError) as context:
            runner.locate_linux_window(exited, 0.01, 0.01)
        self.assertEqual(context.exception.phase, "simulator_exit")

        running = mock.Mock(pid=42)
        running.poll.return_value = None
        empty = subprocess.CompletedProcess(
            ["xdotool"], 1, stdout="", stderr="no windows"
        )
        with (
            mock.patch.object(runner, "owned_process_ids", return_value={42}),
            mock.patch.object(runner, "run_bounded_probe", return_value=empty),
            self.assertRaises(runner.CheckError) as context,
        ):
            runner.locate_linux_window(running, 0.01, 0.01)
        self.assertEqual(context.exception.phase, "capture_window")

    def test_capture_and_gif_encoder_fail_at_their_actual_boundaries(self) -> None:
        runner = load_runner_module()
        window = {"frame_window_id": 88, "window_id": 77, "title": "CoppeliaSim"}
        failed = subprocess.CompletedProcess(
            ["import"], 1, stdout="", stderr="capture failed"
        )
        with (
            mock.patch.object(runner.subprocess, "run", return_value=failed),
            self.assertRaises(runner.CheckError) as context,
        ):
            runner.capture_linux_window(window, self.root / "missing.png", 0.5)
        self.assertEqual(context.exception.phase, "capture_artifact")

        frame = self.root / "frame.png"
        final = self.root / "final.png"
        write_png(frame)
        write_png(final)
        with (
            mock.patch.object(runner.subprocess, "run", return_value=failed),
            self.assertRaises(runner.CheckError) as context,
        ):
            runner.create_linux_animation(
                "normal", "normal-fixture", self.root, window, [frame], final, 4, 0.5
            )
        self.assertEqual(context.exception.phase, "animation_encode")

    def test_vnc_readiness_timeout_reports_the_owned_service_boundary(self) -> None:
        runner = load_runner_module()
        owner = mock.Mock()
        owner.poll.return_value = None
        with (
            mock.patch.object(runner, "host_port_is_occupied", return_value=False),
            self.assertRaises(runner.CheckError) as context,
        ):
            runner.wait_for_internal_port(5900, owner, 0.01, "vnc_readiness")
        self.assertEqual(context.exception.phase, "vnc_readiness")

    def test_compatibility_adjusts_simsubprocess_only_for_rosetta(self) -> None:
        simulator_root = self.root / "CoppeliaSim"
        lua = simulator_root / "lua" / "simSubprocess.lua"
        plugin = simulator_root / "libsimSubprocess.so"
        lua.parent.mkdir(parents=True)
        lua.write_text("fixture", encoding="utf-8")
        plugin.write_text("fixture", encoding="utf-8")

        result = self.run_runner(
            "compatibility", "--mode", "rosetta", "--simulator-root", str(simulator_root)
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        record = json.loads(result.stdout.splitlines()[0])
        self.assertEqual(record["simSubprocess"], "disabled")
        self.assertFalse(lua.exists())
        self.assertFalse(plugin.exists())
        self.assertTrue((simulator_root / "disabled-for-rosetta" / lua.name).is_file())

        native_root = self.root / "native-CoppeliaSim"
        native_lua = native_root / "lua" / "simSubprocess.lua"
        native_lua.parent.mkdir(parents=True)
        native_lua.write_text("fixture", encoding="utf-8")
        native_result = self.run_runner(
            "compatibility", "--mode", "native", "--simulator-root", str(native_root)
        )
        self.assertEqual(native_result.returncode, 0, native_result.stderr)
        self.assertTrue(native_lua.is_file())

    def test_container_entry_rejects_an_incomplete_simulator_package_before_display_start(self) -> None:
        environment = dict(os.environ)
        environment.update(
            {
                "COPPELIASIM_ROOT_DIR": str(self.root / "missing-CoppeliaSim"),
                "ROBORUN_LINUX_GUI_SCENARIO": "normal",
                "ROBORUN_LINUX_GUI_SOURCE_COMMIT": "0123456789abcdef",
                "ROBORUN_LINUX_GUI_COMPATIBILITY_MODE": "native",
                "ROBORUN_LINUX_GUI_COMPATIBILITY_REASON": "fixture",
                "ROBORUN_LINUX_GUI_HOST_NOVNC_URL": "http://127.0.0.1:6088/vnc.html",
                "ROBORUN_LINUX_GUI_IMAGE_ID": "sha256:fixture",
            }
        )

        result = self.run_runner(
            "container-run",
            "--case-dir",
            str(self.root / "container-case"),
            environment=environment,
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("phase=simulator_package", result.stderr)
        self.assertFalse((self.root / "container-case" / "xvfb.log").exists())
        self.assertFalse((self.root / "container-case" / "fluxbox.log").exists())

    def test_novnc_startup_failure_cleans_only_the_named_container(self) -> None:
        docker, command_log = self.make_fake_docker(run_status=9)
        environment = dict(os.environ)
        environment["FAKE_DOCKER_RUN_SECONDS"] = "0.2"

        result = self.run_runner(
            "run",
            "--docker",
            str(docker),
            "--build-dir",
            str(self.root / "build"),
            "--scenario",
            "normal",
            "--novnc-timeout",
            "0.05",
            environment=environment,
        )

        self.assertNotEqual(result.returncode, 0)
        commands = [json.loads(line) for line in command_log.read_text(encoding="utf-8").splitlines()]
        run_command = next(command for command in commands if command[0] == "run")
        owned_name = run_command[run_command.index("--name") + 1]
        source_mount = next(
            run_command[index + 1]
            for index, item in enumerate(run_command[:-1])
            if item == "--volume" and run_command[index + 1].endswith(":/workspace:ro")
        )
        self.assertTrue(owned_name.startswith("roborun-linux-gui-evidence-normal-"))
        self.assertIn("/source/", source_mount)
        self.assertNotEqual(source_mount, f"{PROJECT_ROOT}:/workspace:ro")
        self.assertIn(["rm", "--force", owned_name], commands)
        self.assertNotIn("unrelated-container", command_log.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
