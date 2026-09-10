#!/usr/bin/env python3
"""Build, run and verify RoboRun's Linux Docker GUI evidence."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import signal
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
import uuid
from typing import Any

import gui_evidence_capture as gui_evidence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_SCHEMA = "roborun.linux-gui.evidence.v1"
ENVIRONMENT_SCHEMA = "roborun.linux-gui.environment.v1"
DEFAULT_IMAGE = "roborun-linux-gui:local"
DEFAULT_REMOTE_PORT = 23008
DEFAULT_VNC_PORT = 5908
DEFAULT_NOVNC_PORT = 6088


CheckError = gui_evidence.CheckError
fail = gui_evidence.fail
quote = gui_evidence.quote
sha256 = gui_evidence.sha256
load_json = gui_evidence.load_json


def run_json(command: list[str], phase: str, action: str, timeout: float) -> Any:
    try:
        completed = subprocess.run(
            command, text=True, capture_output=True, check=False, timeout=timeout
        )
    except subprocess.TimeoutExpired:
        fail(phase, f"command exceeded {timeout:g} seconds: {shlex.join(command)}", action)
    except OSError as error:
        fail(phase, f"cannot execute {command[0]}: {error}", action)
    if completed.returncode != 0:
        message = completed.stderr.strip() or completed.stdout.strip() or f"exit status {completed.returncode}"
        fail(phase, message, action)
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        fail(phase, f"command returned invalid JSON: {error}", action)


def validate_port(name: str, port: int) -> None:
    if port < 1 or port > 65535:
        fail("port_configuration", f"{name} port {port} is outside 1..65535", "Choose a valid unused TCP port.")


def host_port_is_occupied(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.2)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def source_commit() -> str:
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        fail("source_provenance", "cannot resolve the current source commit", "Run Linux GUI evidence from a Git checkout.")
    return completed.stdout.strip()


def compatibility_mode(requested: str, docker_version: dict[str, Any]) -> tuple[str, str]:
    if requested != "auto":
        reason = "explicit_command_option"
        return requested, reason
    server_platform = str(docker_version.get("Server", {}).get("Platform", {}).get("Name", ""))
    if platform.system() == "Darwin" and platform.machine() == "arm64" and "Docker Desktop" in server_platform:
        settings_path = Path(
            os.environ.get(
                "ROBORUN_DOCKER_SETTINGS",
                str(
                    Path.home()
                    / "Library"
                    / "Group Containers"
                    / "group.com.docker"
                    / "settings-store.json"
                ),
            )
        )
        try:
            settings = json.loads(settings_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            fail(
                "compatibility_detection",
                f"cannot inspect Docker Desktop Rosetta setting at {settings_path}: {error}",
                "Pass --compatibility-mode rosetta or native explicitly.",
            )
        enabled = settings.get("UseVirtualizationFrameworkRosetta")
        if not isinstance(enabled, bool):
            fail(
                "compatibility_detection",
                "Docker Desktop did not report a boolean Rosetta setting",
                "Pass --compatibility-mode rosetta or native explicitly.",
            )
        if enabled:
            return "rosetta", "docker_desktop_setting_use_virtualization_framework_rosetta=true"
        return "native", "docker_desktop_setting_use_virtualization_framework_rosetta=false"
    return "native", "native_or_non_rosetta_linux_amd64"


def preflight_host(arguments: argparse.Namespace) -> dict[str, Any]:
    docker = Path(arguments.docker)
    if not docker.is_file() or not os.access(docker, os.X_OK):
        fail(
            "docker_unavailable",
            f"Docker executable is missing: {docker}",
            "Install Docker with buildx support or pass its executable with --docker.",
        )
    ports = {
        "remote_api": arguments.remote_port,
        "vnc": arguments.vnc_port,
        "novnc": arguments.novnc_port,
    }
    for name, port in ports.items():
        validate_port(name, port)
    if len(set(ports.values())) != len(ports):
        fail("port_configuration", "Remote API, VNC and noVNC ports must be distinct", "Choose three unused host ports.")
    for name, port in ports.items():
        if host_port_is_occupied(port):
            fail(
                "port_occupied",
                f"host {name} port {port} is already owned by another process",
                f"Stop the existing service or choose a different --{name.replace('_', '-')}-port; Linux GUI evidence will not disturb it.",
            )
    version = run_json(
        [str(docker), "version", "--format", "{{json .}}"],
        "docker_unavailable",
        "Start Docker Desktop or the Docker daemon, then rerun Linux GUI evidence.",
        arguments.docker_timeout,
    )
    info = run_json(
        [str(docker), "info", "--format", "{{json .}}"],
        "docker_unavailable",
        "Start Docker Desktop or the Docker daemon, then rerun Linux GUI evidence.",
        arguments.docker_timeout,
    )
    mode, reason = compatibility_mode(arguments.compatibility_mode, version)
    client = version.get("Client", {})
    server = version.get("Server", {})
    return {
        "schema": ENVIRONMENT_SCHEMA,
        "source_commit": source_commit(),
        "host": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
        },
        "docker": {
            "client_version": client.get("Version"),
            "client_os": client.get("Os"),
            "client_architecture": client.get("Arch"),
            "server_version": server.get("Version"),
            "server_platform": server.get("Platform", {}).get("Name"),
            "server_os": server.get("Os"),
            "server_architecture": server.get("Arch"),
            "operating_system": info.get("OperatingSystem"),
        },
        "container": {"platform": "linux/amd64", "os": "linux", "architecture": "amd64"},
        "compatibility": {"mode": mode, "reason": reason},
        "ports": ports,
    }


def apply_compatibility(mode: str, simulator_root: Path) -> dict[str, Any]:
    disabled_directory = simulator_root / "disabled-for-rosetta"
    candidates = sorted((simulator_root / "lua").glob("simSubprocess*.lua"))
    candidates.extend(sorted(simulator_root.glob("libsimSubprocess*.so")))
    if mode == "native":
        if not candidates:
            fail(
                "compatibility",
                "native mode could not find the retained simSubprocess entry or plugin",
                "Use the complete official CoppeliaSim Edu 4.10 Linux package.",
            )
        return {
            "schema": "roborun.linux-gui.compatibility.v1",
            "mode": mode,
            "simSubprocess": "retained",
            "reason": "native_or_non_rosetta_linux_amd64",
            "files": [str(path.relative_to(simulator_root)) for path in candidates],
        }
    if mode != "rosetta":
        fail("compatibility", f"unsupported compatibility mode: {mode}", "Use native or rosetta.")
    disabled_directory.mkdir(parents=True, exist_ok=True)
    moved: list[str] = []
    for path in candidates:
        destination = disabled_directory / path.name
        if destination.exists():
            destination.unlink()
        shutil.move(str(path), destination)
        moved.append(str(destination.relative_to(simulator_root)))
    existing = sorted(disabled_directory.glob("*simSubprocess*"))
    if not existing:
        fail(
            "compatibility",
            "Rosetta mode could not find the simSubprocess entry or plugin",
            "Use the complete official CoppeliaSim Edu 4.10 Linux package.",
        )
    return {
        "schema": "roborun.linux-gui.compatibility.v1",
        "mode": mode,
        "simSubprocess": "disabled",
        "reason": "darwin_arm64_docker_desktop_linux_amd64_hang",
        "files": moved or [str(path.relative_to(simulator_root)) for path in existing],
    }


def verify_linux_case(scenario: str, case_directory: Path) -> dict[str, Any]:
    environment_path = case_directory / "environment.json"
    process_path = case_directory / "process.json"
    container_log = case_directory / "container.log"
    environment = load_json(environment_path, "environment")
    process = load_json(process_path, "process_result")
    if environment.get("schema") != ENVIRONMENT_SCHEMA:
        fail("environment", "case has no Linux GUI environment record", "Use evidence produced by the Linux GUI evidence runner.")
    container = environment.get("container", {})
    if container.get("os") != "linux" or container.get("architecture") != "amd64":
        fail("container_environment", "case was not produced by a Linux amd64 container", "Run the declared linux/amd64 image.")
    host = environment.get("host", {})
    docker = environment.get("docker", {})
    if not host.get("machine") or not docker.get("server_version") or not docker.get("server_platform"):
        fail("host_provenance", "host architecture or Docker version evidence is incomplete", "Keep host-environment.json from the same Linux GUI evidence run.")
    simulator = environment.get("simulator", {})
    digest = simulator.get("archive_sha256")
    if simulator.get("version") != "4.10.0 rev0" or not isinstance(digest, str) or len(digest) != 64:
        fail("simulator_provenance", "CoppeliaSim version or archive digest is incomplete", "Rebuild from the fixed official Edu 4.10 archive.")
    display = environment.get("display", {})
    if not display.get("xvfb") or not display.get("managed_window") or not display.get("renderer"):
        fail("display_evidence", "Xvfb, renderer or managed-window evidence is incomplete", "Inspect xvfb.log, fluxbox.log and window evidence.")
    window = environment.get("captured_window", {})
    if (
        not isinstance(window.get("owner_pid"), int)
        or not isinstance(window.get("frame_window_id"), int)
        or "CoppeliaSim" not in str(window.get("title", ""))
        or window.get("capture_width", 0) < 640
        or window.get("capture_height", 0) < 480
    ):
        fail("window_evidence", "owned managed CoppeliaSim window evidence is incomplete", "Inspect X11 PID, frame and geometry records.")
    novnc = environment.get("novnc", {})
    if novnc.get("reachable_from_host") is not True or not str(novnc.get("url", "")).startswith("http://127.0.0.1:"):
        fail("novnc_evidence", "noVNC was not proven reachable from the host", "Inspect noVNC logs and published port ownership.")
    compatibility = environment.get("compatibility", {})
    expected_plugin_state = "disabled" if compatibility.get("mode") == "rosetta" else "retained"
    if compatibility.get("simSubprocess") != expected_plugin_state or not compatibility.get("reason"):
        fail("compatibility", "simSubprocess evidence does not match the compatibility mode", "Rebuild or rerun with the correct compatibility profile.")
    released = process.get("ports_released", {})
    if process.get("container_removed") is not True or any(released.get(name) is not True for name in ("remote_api", "vnc", "novnc")):
        fail("cleanup", "container removal or published-port release is incomplete", "Remove the owned container and release all three Linux GUI evidence ports.")
    service_statuses = (
        "xvfb_exit_status",
        "window_manager_exit_status",
        "x11vnc_exit_status",
        "novnc_exit_status",
        "simulator_exit_status",
    )
    if any(not isinstance(process.get(name), int) for name in service_statuses):
        fail(
            "process_evidence",
            "owned GUI service exit-status evidence is incomplete",
            "Retain process.json after all owned container services finish.",
        )
    if not container_log.is_file():
        fail("container_log", "container log is missing", "Retain Docker stdout and stderr beside the case evidence.")

    manifest = gui_evidence.verify_case(scenario, case_directory)
    artifacts = manifest["artifacts"]
    recorded_paths = {record["path"] for record in artifacts.values()}
    for path in sorted(case_directory.iterdir()):
        if not path.is_file() or path.name == "manifest.json" or path.name in recorded_paths:
            continue
        artifacts[path.name.replace(".", "_")] = {
            "path": path.name,
            "sha256": sha256(path),
            "bytes": path.stat().st_size,
        }
    manifest.update(
        {
            "schema": EVIDENCE_SCHEMA,
            "environment": {
                "image_id": environment.get("docker", {}).get("image_id"),
                "container_architecture": container.get("architecture"),
                "archive_sha256": digest,
                "renderer": display.get("renderer"),
                "compatibility_mode": compatibility.get("mode"),
                "novnc_url": novnc.get("url"),
            },
        }
    )
    (case_directory / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return manifest


def build_image(arguments: argparse.Namespace, environment: dict[str, Any], build_directory: Path) -> dict[str, Any]:
    dockerfile = PROJECT_ROOT / "docker" / "linux-gui" / "Dockerfile"
    log_path = build_directory / "image-build.log"
    command = [
        str(arguments.docker),
        "buildx",
        "build",
        "--platform",
        "linux/amd64",
        "--load",
        "--file",
        str(dockerfile),
        "--tag",
        arguments.image,
        str(dockerfile.parent),
    ]
    with log_path.open("w", encoding="utf-8") as output:
        output.write("COMMAND " + " ".join(command) + "\n")
        output.flush()
        try:
            completed = subprocess.run(
                command,
                stdout=output,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=arguments.image_build_timeout,
            )
        except subprocess.TimeoutExpired:
            fail(
                "image_build_timeout",
                f"Docker image build exceeded {arguments.image_build_timeout:g} seconds",
                f"Inspect {log_path}, Docker networking and {dockerfile}.",
            )
        except OSError as error:
            fail("image_build", f"cannot execute Docker build: {error}", f"Inspect {dockerfile} and Docker availability.")
    if completed.returncode != 0:
        fail(
            "image_build",
            f"Docker image build exited with status {completed.returncode}",
            f"Inspect {log_path} and {dockerfile}.",
        )
    inspected = run_json(
        [str(arguments.docker), "image", "inspect", arguments.image],
        "image_inspect",
        "Inspect the locally built Linux GUI evidence image.",
        arguments.docker_timeout,
    )
    if not isinstance(inspected, list) or not inspected:
        fail("image_inspect", "Docker returned no image metadata", "Rebuild the local Linux GUI evidence image.")
    image = inspected[0]
    if image.get("Os") != "linux" or image.get("Architecture") != "amd64":
        fail(
            "image_architecture",
            f"Linux GUI evidence requires linux/amd64, image reports {image.get('Os')}/{image.get('Architecture')}",
            "Enable linux/amd64 emulation or build on native x86_64 Linux.",
        )
    return image


def wait_for_observation(path: Path, process: subprocess.Popen[Any], timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.is_file():
            return load_json(path, "novnc_host")
        status = process.poll()
        if status is not None:
            fail("container_run", f"container exited with status {status} before noVNC readiness", "Inspect the retained container and service logs.")
        time.sleep(0.05)
    fail("novnc_host", f"noVNC readiness exceeded {timeout:g} seconds", "Inspect xvfb.log, fluxbox.log, vnc.log and novnc.log.")


def probe_novnc(url: str, timeout: float) -> None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            if response.status != 200:
                fail("novnc_host", f"noVNC returned HTTP {response.status}", "Inspect noVNC and Docker port publishing.")
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        fail("novnc_host", f"cannot reach {url}: {error}", "Inspect noVNC and Docker port publishing.")


def remove_owned_container(docker: Path, container_name: str, timeout: float) -> bool:
    try:
        subprocess.run(
            [str(docker), "stop", "--time", "10", container_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=timeout,
        )
        removed = subprocess.run(
            [str(docker), "rm", "--force", container_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return False
    return removed.returncode == 0


def prepare_source_snapshot(build_directory: Path, commit: str, timeout: float) -> Path:
    snapshot_parent = build_directory / "source"
    snapshot = snapshot_parent / commit
    marker = snapshot / ".roborun-source-commit"
    if marker.is_file() and marker.read_text(encoding="utf-8").strip() == commit:
        return snapshot
    snapshot_parent.mkdir(parents=True, exist_ok=True)
    if snapshot.exists():
        shutil.rmtree(snapshot)
    snapshot.mkdir()
    archive = snapshot_parent / f"{commit}.tar"
    try:
        subprocess.run(
            ["git", "archive", "--format=tar", "--output", str(archive), commit],
            cwd=PROJECT_ROOT,
            check=True,
            timeout=timeout,
        )
        subprocess.run(
            ["tar", "-xf", str(archive), "-C", str(snapshot)],
            check=True,
            timeout=timeout,
        )
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        fail(
            "source_snapshot",
            f"cannot materialize source commit {commit}: {error}",
            "Inspect Git, tar and the dedicated Linux GUI evidence build directory.",
        )
    finally:
        archive.unlink(missing_ok=True)
    marker.write_text(commit + "\n", encoding="utf-8")
    return snapshot


def wait_for_host_ports_release(ports: dict[str, int], timeout: float = 15.0) -> dict[str, bool]:
    deadline = time.monotonic() + timeout
    result = {name: False for name in ports}
    while time.monotonic() < deadline:
        result = {name: not host_port_is_occupied(port) for name, port in ports.items()}
        if all(result.values()):
            return result
        time.sleep(0.1)
    return result


def run_container_scenario(
    arguments: argparse.Namespace,
    base_environment: dict[str, Any],
    image: dict[str, Any],
    check_directory: Path,
    source_snapshot: Path,
    scenario: str,
) -> dict[str, Any]:
    case_directory = check_directory / scenario
    case_directory.mkdir()
    for name, port in base_environment["ports"].items():
        if host_port_is_occupied(port):
            fail(
                "port_occupied",
                f"host {name} port {port} became occupied before {scenario}",
                "Stop the external owner or choose another Linux GUI evidence port; the runner will not reuse it.",
            )
    container_name = f"roborun-linux-gui-evidence-{scenario}-{uuid.uuid4().hex[:8]}"
    container_log_path = case_directory / "container.log"
    observation_path = case_directory / "observation-ready.json"
    confirmation_path = case_directory / "observation-confirmed.json"
    ports = base_environment["ports"]
    build_mount = arguments.build_dir.resolve() / "container-build"
    build_mount.mkdir(parents=True, exist_ok=True)
    host_environment = {
        **base_environment,
        "docker": {**base_environment["docker"], "image_id": image.get("Id")},
    }
    (case_directory / "host-environment.json").write_text(
        json.dumps(host_environment, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    command = [
        str(arguments.docker),
        "run",
        "--name",
        container_name,
        "--platform",
        "linux/amd64",
        "--shm-size",
        "1g",
        "--publish",
        f"127.0.0.1:{ports['remote_api']}:23000",
        "--publish",
        f"127.0.0.1:{ports['vnc']}:5900",
        "--publish",
        f"127.0.0.1:{ports['novnc']}:6080",
        "--volume",
        f"{source_snapshot}:/workspace:ro",
        "--volume",
        f"{build_mount}:/build",
        "--volume",
        f"{case_directory}:/evidence",
        "--env",
        f"ROBORUN_LINUX_GUI_SCENARIO={scenario}",
        "--env",
        f"ROBORUN_LINUX_GUI_SOURCE_COMMIT={base_environment['source_commit']}",
        "--env",
        f"ROBORUN_LINUX_GUI_COMPATIBILITY_MODE={base_environment['compatibility']['mode']}",
        "--env",
        f"ROBORUN_LINUX_GUI_COMPATIBILITY_REASON={base_environment['compatibility']['reason']}",
        "--env",
        f"ROBORUN_LINUX_GUI_HOST_NOVNC_URL=http://127.0.0.1:{ports['novnc']}/vnc.html",
        "--env",
        f"ROBORUN_LINUX_GUI_IMAGE_ID={image.get('Id')}",
        arguments.image,
        "--source-build-timeout",
        str(arguments.source_build_timeout),
        "--command-timeout",
        str(arguments.command_timeout),
    ]
    process: subprocess.Popen[Any] | None = None
    run_error: CheckError | None = None
    removed = False
    with container_log_path.open("w", encoding="utf-8") as output:
        output.write("COMMAND " + " ".join(command) + "\n")
        output.flush()
        try:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, text=True)
            observation = wait_for_observation(observation_path, process, arguments.novnc_timeout)
            url = str(observation.get("url", ""))
            expected_url = f"http://127.0.0.1:{ports['novnc']}/vnc.html"
            if url != expected_url:
                fail("novnc_host", f"container advertised unexpected noVNC URL {url!r}", "Use the published Linux GUI evidence noVNC port.")
            probe_novnc(url, arguments.novnc_timeout)
            confirmation_path.write_text(
                json.dumps({"schema": "roborun.linux-gui.host-observation.v1", "url": url, "observed_at": datetime.now(timezone.utc).isoformat()}) + "\n",
                encoding="utf-8",
            )
            status = process.wait(timeout=arguments.scenario_timeout)
            if status != 0:
                fail("container_run", f"{scenario} container exited with status {status}", "Inspect the retained case logs and failure.json.")
        except subprocess.TimeoutExpired:
            run_error = CheckError("container_timeout", f"{scenario} exceeded {arguments.scenario_timeout:g} seconds", "Inspect Runtime, simulator and capture logs.")
        except CheckError as error:
            run_error = error
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            removed = remove_owned_container(
                Path(arguments.docker), container_name, arguments.docker_timeout
            )
    released = wait_for_host_ports_release(ports)
    process_path = case_directory / "process.json"
    if process_path.is_file():
        process_record = load_json(process_path, "process_result")
    else:
        process_record = {"runtime_exit_status": None, "cleanup_succeeded": False}
    process_record.update(
        {
            "container_name": container_name,
            "container_removed": removed,
            "ports_released": released,
            "cleanup_succeeded": bool(process_record.get("cleanup_succeeded")) and removed and all(released.values()),
        }
    )
    process_path.write_text(json.dumps(process_record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if run_error is not None:
        failure = {
            "schema": "roborun.linux-gui.failure.v1",
            "phase": run_error.phase,
            "message": str(run_error),
            "action": run_error.action,
            "container_name": container_name,
            "container_removed": removed,
            "ports_released": released,
        }
        (case_directory / "host-failure.json").write_text(json.dumps(failure, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        raise run_error
    return verify_linux_case(scenario, case_directory)


def run_check(arguments: argparse.Namespace) -> Path:
    environment = preflight_host(arguments)
    build_directory = arguments.build_dir.resolve()
    build_directory.mkdir(parents=True, exist_ok=True)
    source_snapshot = prepare_source_snapshot(
        build_directory, environment["source_commit"], arguments.source_snapshot_timeout
    )
    image = build_image(arguments, environment, build_directory)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + environment["source_commit"][:8]
    check_directory = build_directory / "evidence" / run_id
    check_directory.mkdir(parents=True)
    scenarios = ("normal", "estop") if arguments.scenario == "all" else (arguments.scenario,)
    manifests: dict[str, dict[str, Any]] = {}
    for scenario in scenarios:
        manifests[scenario] = run_container_scenario(
            arguments,
            environment,
            image,
            check_directory,
            source_snapshot,
            scenario,
        )
    combined = {
        "schema": EVIDENCE_SCHEMA,
        "run_id": run_id,
        "source_commit": environment["source_commit"],
        "scenario_order": list(scenarios),
        "image": {"id": image.get("Id"), "os": image.get("Os"), "architecture": image.get("Architecture")},
        "cases": {
            scenario: {"manifest": f"{scenario}/manifest.json", "sha256": sha256(check_directory / scenario / "manifest.json")}
            for scenario in scenarios
        },
        "visual_review": "required",
    }
    (check_directory / "manifest.json").write_text(json.dumps(combined, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return check_directory


def require_container_file(path: Path, phase: str, action: str, executable: bool = False) -> None:
    if not path.is_file() or (executable and not os.access(path, os.X_OK)):
        fail(phase, f"required simulator file is missing: {path}", action)


def require_container_command(name: str) -> str:
    path = shutil.which(name)
    if path is None:
        fail("container_dependency", f"required command is missing: {name}", "Rebuild the declared Linux GUI evidence Dockerfile.")
    return path


def container_package_preflight(simulator_root: Path) -> dict[str, Any]:
    require_container_file(
        simulator_root / "coppeliaSim.sh",
        "simulator_package",
        "Rebuild from the complete official CoppeliaSim Edu 4.10 Ubuntu 22.04 archive.",
        executable=True,
    )
    for model in (
        simulator_root / "models" / "robots" / "non-mobile" / "UR5.ttm",
        simulator_root / "models" / "components" / "grippers" / "RG2.ttm",
    ):
        require_container_file(
            model,
            "simulator_package",
            "Rebuild from the complete official CoppeliaSim Edu 4.10 Ubuntu 22.04 archive.",
        )
    archive_digest_path = simulator_root.parent / "CoppeliaSim.archive.sha256"
    archive_url_path = simulator_root.parent / "CoppeliaSim.archive.url"
    require_container_file(
        archive_digest_path,
        "simulator_package",
        "Use the Linux GUI evidence Dockerfile so archive integrity and provenance are recorded before extraction.",
    )
    require_container_file(
        archive_url_path,
        "simulator_package",
        "Use the Linux GUI evidence Dockerfile so the fixed official archive source is recorded.",
    )
    digest = archive_digest_path.read_text(encoding="utf-8").strip()
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest.lower()):
        fail("simulator_package", "recorded CoppeliaSim archive SHA-256 is invalid", "Rebuild the Linux GUI evidence image from a complete archive.")
    return {
        "version": "4.10.0 rev0",
        "archive_url": archive_url_path.read_text(encoding="utf-8").strip(),
        "archive_sha256": digest,
        "root": str(simulator_root),
    }


def run_logged_container(
    command: list[str], log_path: Path, phase: str, action: str, timeout: float
) -> None:
    with log_path.open("a", encoding="utf-8") as output:
        output.write("COMMAND " + shlex.join(command) + "\n")
        output.flush()
        try:
            completed = subprocess.run(
                command,
                stdout=output,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            fail(phase, f"command exceeded {timeout:g} seconds: {shlex.join(command)}", action)
    if completed.returncode != 0:
        fail(phase, f"command exited with status {completed.returncode}: {shlex.join(command)}", action)


def start_logged_process(command: list[str], log_path: Path, environment: dict[str, str] | None = None) -> tuple[subprocess.Popen[Any], Any]:
    output = log_path.open("a", encoding="utf-8")
    output.write("COMMAND " + shlex.join(command) + "\n")
    output.flush()
    try:
        process = subprocess.Popen(
            command,
            stdout=output,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            text=True,
            env=environment,
            start_new_session=True,
        )
    except OSError:
        output.close()
        raise
    return process, output


def stop_owned_process_group(process: subprocess.Popen[Any] | None, timeout: float = 10.0) -> bool:
    if process is None or process.poll() is not None:
        return True
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return True
    try:
        process.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False


def wait_for_condition(
    predicate: Any,
    process: subprocess.Popen[Any] | None,
    timeout: float,
    phase: str,
    message: str,
    action: str,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        if process is not None and process.poll() is not None:
            fail(phase, f"owned process exited with status {process.returncode}: {message}", action)
        time.sleep(0.1)
    fail(phase, message, action)


def owned_process_ids(root_pid: int, command_timeout: float = 5.0) -> set[int]:
    completed = run_bounded_probe(
        ["ps", "-eo", "pid=,ppid="],
        command_timeout,
        "window_ownership",
        "Inspect procps installation and simulator.log.",
    )
    if completed.returncode != 0:
        fail("window_ownership", "cannot read the container process tree", "Inspect procps installation and simulator.log.")
    children: dict[int, list[int]] = {}
    for line in completed.stdout.splitlines():
        fields = line.split()
        if len(fields) != 2 or not all(field.isdigit() for field in fields):
            continue
        pid, parent = map(int, fields)
        children.setdefault(parent, []).append(pid)
    owned = {root_pid}
    pending = [root_pid]
    while pending:
        parent = pending.pop()
        for child in children.get(parent, []):
            if child not in owned:
                owned.add(child)
                pending.append(child)
    return owned


def parse_shell_record(text: str) -> dict[str, int]:
    result: dict[str, int] = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        if value.lstrip("-").isdigit():
            result[key] = int(value)
    return result


def frame_extents(
    window_id: int, command_timeout: float = 5.0
) -> tuple[int, int, int, int]:
    completed = run_bounded_probe(
        ["xprop", "-id", str(window_id), "_NET_FRAME_EXTENTS"],
        command_timeout,
        "window_ownership",
        "Inspect X11 frame properties and Fluxbox readiness.",
    )
    if completed.returncode != 0 or "=" not in completed.stdout:
        return 0, 0, 0, 0
    values = [value.strip() for value in completed.stdout.split("=", 1)[1].split(",")]
    if len(values) != 4 or not all(value.isdigit() for value in values):
        return 0, 0, 0, 0
    return tuple(int(value) for value in values)  # type: ignore[return-value]


def frame_window_id(window_id: int, command_timeout: float = 5.0) -> int:
    completed = run_bounded_probe(
        ["xwininfo", "-id", str(window_id), "-tree"],
        command_timeout,
        "window_ownership",
        "Inspect X11 window properties and xwininfo installation.",
    )
    if completed.returncode != 0:
        fail("window_ownership", f"cannot inspect the frame parent of window {window_id}", "Inspect X11 window properties and xwininfo installation.")
    for line in completed.stdout.splitlines():
        if "Parent window id:" not in line:
            continue
        raw = line.split("Parent window id:", 1)[1].strip().split()[0]
        try:
            parent = int(raw, 16 if raw.startswith("0x") else 10)
        except ValueError:
            break
        if parent > 0:
            return parent
    fail("window_ownership", f"window {window_id} has no inspectable managed frame parent", "Inspect Fluxbox reparenting and X11 properties.")


def locate_linux_window(
    simulator: subprocess.Popen[Any],
    timeout: float,
    command_timeout: float = 5.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    rejection_reasons: list[str] = []
    while time.monotonic() < deadline:
        if simulator.poll() is not None:
            fail("simulator_exit", f"CoppeliaSim exited with status {simulator.returncode} before window discovery", "Inspect simulator.log and compatibility.json.")
        owned = owned_process_ids(simulator.pid, command_timeout)
        search = run_bounded_probe(
            ["xdotool", "search", "--onlyvisible", "--class", "CoppeliaSim"],
            command_timeout,
            "capture_window",
            "Inspect the owned CoppeliaSim process and X11 display.",
        )
        candidates: list[dict[str, Any]] = []
        for raw in search.stdout.splitlines():
            if not raw.isdigit():
                continue
            window_id = int(raw)
            pid_result = run_bounded_probe(
                ["xdotool", "getwindowpid", raw],
                command_timeout,
                "capture_window",
                "Inspect X11 window PID properties.",
            )
            owner_pid = int(pid_result.stdout.strip()) if pid_result.stdout.strip().isdigit() else -1
            if owner_pid not in owned:
                rejection_reasons.append(f"window {window_id} belongs to external pid {owner_pid}")
                continue
            window_type = run_bounded_probe(
                ["xprop", "-id", raw, "_NET_WM_WINDOW_TYPE"],
                command_timeout,
                "capture_window",
                "Inspect the candidate X11 window type.",
            ).stdout
            if "_NET_WM_WINDOW_TYPE_NORMAL" not in window_type:
                rejection_reasons.append(f"window {window_id} is not a normal top-level window")
                continue
            geometry_result = run_bounded_probe(
                ["xdotool", "getwindowgeometry", "--shell", raw],
                command_timeout,
                "capture_window",
                "Inspect the candidate X11 window geometry.",
            )
            geometry = parse_shell_record(geometry_result.stdout)
            width = geometry.get("WIDTH", 0)
            height = geometry.get("HEIGHT", 0)
            if width < 800 or height < 600:
                rejection_reasons.append(f"window {window_id} is implausibly small: {width}x{height}")
                continue
            title_result = run_bounded_probe(
                ["xdotool", "getwindowname", raw],
                command_timeout,
                "capture_window",
                "Inspect the candidate X11 window title.",
            )
            title = title_result.stdout.strip()
            if "CoppeliaSim" not in title:
                rejection_reasons.append(f"window {window_id} has unexpected title {title!r}")
                continue
            left, right, top, bottom = frame_extents(window_id, command_timeout)
            candidates.append(
                {
                    "window_id": window_id,
                    "frame_window_id": frame_window_id(window_id, command_timeout),
                    "owner_pid": owner_pid,
                    "title": title,
                    "managed": True,
                    "client_width": width,
                    "client_height": height,
                    "capture_x": geometry.get("X", 0) - left,
                    "capture_y": geometry.get("Y", 0) - top,
                    "capture_width": width + left + right,
                    "capture_height": height + top + bottom,
                    "frame_extents": [left, right, top, bottom],
                }
            )
        if candidates:
            return max(candidates, key=lambda candidate: candidate["capture_width"] * candidate["capture_height"])
        time.sleep(0.2)
    detail = rejection_reasons[-1] if rejection_reasons else "no visible CoppeliaSim window matched the owned process tree"
    fail("capture_window", detail, "Inspect simulator.log, xvfb.log, fluxbox.log and X11 window properties.")


def capture_linux_window(window: dict[str, Any], path: Path, timeout: float) -> None:
    try:
        completed = subprocess.run(
            [
                "import",
                "-display",
                os.environ.get("DISPLAY", ":99"),
                "-window",
                str(window["frame_window_id"]),
                str(path),
            ],
            text=True,
            capture_output=True,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        fail("capture_timeout", f"window capture exceeded {timeout:g} seconds", "Inspect Xvfb and ImageMagick logs.")
    if completed.returncode != 0 or not path.is_file():
        fail("capture_artifact", completed.stderr.strip() or "ImageMagick did not create a PNG", "Inspect Xvfb, window geometry and ImageMagick.")
    width, height = gui_evidence.png_dimensions(path)
    if width < 640 or height < 480:
        fail("capture_artifact", f"captured window is too small: {width}x{height}", "Expose the normal managed CoppeliaSim window.")


class LinuxWindowRecorder:
    def __init__(self, window: dict[str, Any], directory: Path, frames_per_second: float, capture_timeout: float) -> None:
        self.window = window
        self.directory = directory
        self.frames_per_second = frames_per_second
        self.capture_timeout = capture_timeout
        self.frames: list[Path] = []
        self.error: CheckError | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _record(self) -> None:
        interval = 1.0 / self.frames_per_second
        while not self._stop.is_set():
            frame = self.directory / f"frame-{len(self.frames):05d}.png"
            try:
                capture_linux_window(self.window, frame, self.capture_timeout)
                self.frames.append(frame)
            except CheckError as error:
                self.error = error
                self._stop.set()
                return
            self._stop.wait(interval)

    def start(self) -> None:
        self.directory.mkdir()
        self._thread = threading.Thread(target=self._record, name="linux-gui-evidence-linux-window-recorder", daemon=True)
        self._thread.start()

    def stop(self) -> list[Path]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(5.0, self.capture_timeout + 1.0))
            if self._thread.is_alive():
                fail("animation_capture", "Linux window recorder did not stop within its timeout", "Inspect X11 and retained frames.")
        if self.error is not None:
            raise self.error
        return list(self.frames)


def create_linux_animation(
    scenario: str,
    scenario_id: str,
    case_directory: Path,
    window: dict[str, Any],
    frames: list[Path],
    final_screenshot: Path,
    frames_per_second: float,
    timeout: float,
) -> dict[str, Any]:
    all_frames = [*frames, final_screenshot]
    if len(all_frames) < 2:
        fail("animation_capture", "Linux window recording produced fewer than two frames", "Inspect the retained frame directory.")
    animation_path = case_directory / f"{scenario_id}.gif"
    delay = max(1, round(100 / frames_per_second))
    command = ["convert", "-delay", str(delay), "-loop", "0"]
    for frame in all_frames:
        command.extend([str(frame), "-resize", "1200x1200>"])
    command.append(str(animation_path))
    try:
        completed = subprocess.run(command, text=True, capture_output=True, check=False, timeout=timeout)
    except subprocess.TimeoutExpired:
        fail("animation_encode", f"GIF encoding exceeded {timeout:g} seconds", "Inspect retained PNG frames and reduce --gif-fps.")
    if completed.returncode != 0 or not animation_path.is_file():
        fail("animation_encode", completed.stderr.strip() or "ImageMagick did not create a GIF", "Inspect retained PNG frames and ImageMagick.")
    width, height, frame_count = gui_evidence.gif_properties(animation_path)
    if frame_count < 2:
        fail("animation_encode", "encoded GIF has fewer than two frames", "Inspect the captured motion frames.")
    metadata = {
        "schema": "roborun.gui.animation.v1",
        "scenario": scenario,
        "scenario_id": scenario_id,
        "file": animation_path.name,
        "frame_count": frame_count,
        "frames_per_second": frames_per_second,
        "width": width,
        "height": height,
        "window_id": window["window_id"],
        "window_title": window["title"],
    }
    (case_directory / "animation.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


def wait_for_internal_port(port: int, owner: subprocess.Popen[Any], timeout: float, phase: str) -> None:
    wait_for_condition(
        lambda: host_port_is_occupied(port),
        owner,
        timeout,
        phase,
        f"TCP port {port} did not become ready within {timeout:g} seconds",
        "Inspect the corresponding owned service log.",
    )


def run_bounded_probe(
    command: list[str], timeout: float, phase: str, action: str
) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            text=True,
            capture_output=True,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        fail(
            phase,
            f"command exceeded {timeout:g} seconds: {shlex.join(command)}",
            action,
        )
    except OSError as error:
        fail(phase, f"cannot execute {command[0]}: {error}", action)


def display_is_ready(display: str, command_timeout: float) -> bool:
    completed = run_bounded_probe(
        ["xdpyinfo", "-display", display],
        command_timeout,
        "xvfb_readiness",
        "Inspect the Xvfb process, log and DISPLAY value.",
    )
    return completed.returncode == 0


def remote_api_is_ready(
    simulator_root: Path, port: int, command_timeout: float
) -> bool:
    client_root = (
        simulator_root
        / "programming"
        / "zmqRemoteApi"
        / "clients"
        / "python"
        / "src"
    )
    probe = """
import sys
sys.path.insert(0, sys.argv[1])
import zmq
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient(host='127.0.0.1', port=int(sys.argv[2]))
client.initialTimeout = 1
try:
    client.getObject('sim').getSimulationState()
finally:
    client.socket.setsockopt(zmq.LINGER, 0)
    client.socket.close()
    client.context.term()
""".strip()
    completed = run_bounded_probe(
        ["python3", "-c", probe, str(client_root), str(port)],
        command_timeout,
        "remote_api_readiness",
        "Inspect simulator.log and the CoppeliaSim ZeroMQ Remote API service.",
    )
    return completed.returncode == 0


def renderer_name(command_timeout: float = 5.0) -> str:
    completed = run_bounded_probe(
        ["glxinfo", "-B"],
        command_timeout,
        "opengl",
        "Inspect Xvfb GLX support and Mesa packages.",
    )
    if completed.returncode != 0:
        fail("opengl", completed.stderr.strip() or "glxinfo failed", "Inspect Xvfb GLX support and Mesa packages.")
    for line in completed.stdout.splitlines():
        if "OpenGL renderer string:" in line:
            renderer = line.split(":", 1)[1].strip()
            if renderer:
                return renderer
    fail("opengl", "glxinfo did not report an OpenGL renderer", "Install Mesa software rendering support.")


def window_manager_is_ready(command_timeout: float = 5.0) -> bool:
    completed = run_bounded_probe(
        ["xprop", "-root", "_NET_SUPPORTING_WM_CHECK"],
        command_timeout,
        "window_manager_readiness",
        "Inspect the Fluxbox process and X11 root-window properties.",
    )
    if completed.returncode != 0:
        return False
    prefix = "_NET_SUPPORTING_WM_CHECK(WINDOW): window id # "
    for line in completed.stdout.splitlines():
        if not line.startswith(prefix):
            continue
        try:
            return int(line.removeprefix(prefix).strip(), 0) > 0
        except ValueError:
            return False
    return False


def wait_for_host_confirmation(path: Path, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.is_file():
            return load_json(path, "novnc_host")
        time.sleep(0.1)
    fail("novnc_host", "host did not confirm the published noVNC endpoint", "Inspect Docker port publishing and the host runner.")


def container_runtime_command(
    scenario: str,
    scenario_id: str,
    ready_path: Path,
    continue_path: Path,
    scene_path: Path,
    simulator_root: Path,
    evidence_timeout: float,
) -> list[str]:
    command = [
        "/build/core/roborun",
        "run",
        "--backend",
        "coppeliasim",
        "--coppeliasim-port",
        "23000",
        "--robot-config",
        "/workspace/examples/config/ur5_robot.json",
        "--points-config",
        "/workspace/examples/config/ur5_points.json",
        "--io-config",
        "/workspace/examples/config/workcell_io.json",
        "--tool-config",
        "/workspace/examples/config/tool_feedback.json",
        "--workcell-config",
        "/workspace/examples/config/workcell.json",
        "--resources",
        str(simulator_root),
        "--scene",
        str(scene_path),
        "--program",
        "/workspace/examples/tasks/pick_place.task",
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
        str(round(evidence_timeout * 1000)),
    ]
    if scenario == "estop":
        command.extend(["--estop-after-move-polls", "20"])
    return command


def container_run(arguments: argparse.Namespace) -> None:
    simulator_root = Path(os.environ.get("COPPELIASIM_ROOT_DIR", "/opt/CoppeliaSim"))
    simulator_provenance = container_package_preflight(simulator_root)
    for command in (
        "Xvfb",
        "fluxbox",
        "x11vnc",
        "websockify",
        "xdpyinfo",
        "xprop",
        "xwininfo",
        "xdotool",
        "glxinfo",
        "import",
        "convert",
        "cmake",
        "git",
    ):
        require_container_command(command)
    scenario = os.environ.get("ROBORUN_LINUX_GUI_SCENARIO", "")
    if scenario not in ("normal", "estop"):
        fail("container_environment", f"invalid scenario {scenario!r}", "Set ROBORUN_LINUX_GUI_SCENARIO to normal or estop.")
    case_directory = arguments.case_dir.resolve()
    case_directory.mkdir(parents=True, exist_ok=True)
    os.makedirs(os.environ.get("XDG_RUNTIME_DIR", "/tmp/runtime-roborun"), mode=0o700, exist_ok=True)
    mode = os.environ.get("ROBORUN_LINUX_GUI_COMPATIBILITY_MODE", "native")
    compatibility = apply_compatibility(mode, simulator_root)
    compatibility["reason"] = os.environ.get("ROBORUN_LINUX_GUI_COMPATIBILITY_REASON", compatibility["reason"])
    (case_directory / "compatibility.json").write_text(json.dumps(compatibility, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    for name in ("remote_api", "vnc", "novnc"):
        port = {"remote_api": 23000, "vnc": 5900, "novnc": 6080}[name]
        if host_port_is_occupied(port):
            fail("container_port_occupied", f"internal {name} port {port} is already occupied", "Use a fresh Linux GUI evidence container instance.")

    build_log = case_directory / "build.log"
    run_logged_container(
        [
            "cmake",
            "-S",
            "/workspace",
            "-B",
            "/build/core",
            "-DBUILD_TESTING=OFF",
            "-DROBORUN_ENABLE_COPPELIASIM=ON",
            f"-DCOPPELIASIM_ROOT_DIR={simulator_root}",
        ],
        build_log,
        "container_build",
        "Inspect build.log and the mounted source commit.",
        arguments.source_build_timeout,
    )
    run_logged_container(
        ["cmake", "--build", "/build/core", "--target", "roborun", "--parallel", "2"],
        build_log,
        "container_build",
        "Inspect build.log and CoppeliaSim C++ client dependencies.",
        arguments.source_build_timeout,
    )

    xvfb_log = case_directory / "xvfb.log"
    window_manager_log = case_directory / "fluxbox.log"
    vnc_log = case_directory / "vnc.log"
    novnc_log = case_directory / "novnc.log"
    simulator_log = case_directory / "simulator.log"
    runtime_log = case_directory / "runtime.log"
    service_outputs: list[Any] = []
    xvfb: subprocess.Popen[Any] | None = None
    window_manager: subprocess.Popen[Any] | None = None
    vnc: subprocess.Popen[Any] | None = None
    novnc: subprocess.Popen[Any] | None = None
    simulator: subprocess.Popen[Any] | None = None
    runtime: subprocess.Popen[Any] | None = None
    runtime_status: int | None = None
    recorder: LinuxWindowRecorder | None = None
    scenario_error: CheckError | None = None
    capture_error: CheckError | None = None
    cleanup_succeeded = True
    scenario_id = gui_evidence.scenario_identifier(scenario)
    ready_path = case_directory / "ready.json"
    continue_path = case_directory / "continue.json"
    scene_path = case_directory / "workcell.ttt"
    process_path = case_directory / "process.json"
    environment_path = case_directory / "environment.json"
    environment: dict[str, Any] = {
        "schema": ENVIRONMENT_SCHEMA,
        "source_commit": os.environ.get("ROBORUN_LINUX_GUI_SOURCE_COMMIT"),
        "launch_mode": "gui",
        "scenario": scenario,
        "scenario_id": scenario_id,
        "container": {"os": "linux", "architecture": platform.machine().replace("x86_64", "amd64"), "platform": "linux/amd64"},
        "docker": {"image_id": os.environ.get("ROBORUN_LINUX_GUI_IMAGE_ID")},
        "simulator": simulator_provenance,
        "compatibility": compatibility,
        "ports": {"remote_api": 23000, "vnc": 5900, "novnc": 6080},
        "started_at": datetime.now(timezone.utc).isoformat(),
    }
    host_environment_path = case_directory / "host-environment.json"
    if host_environment_path.is_file():
        host_environment = load_json(host_environment_path, "host_provenance")
        environment["host"] = host_environment.get("host", {})
        environment["docker"] = host_environment.get("docker", environment["docker"])
    environment_path.write_text(json.dumps(environment, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    try:
        xvfb, output = start_logged_process(
            ["Xvfb", os.environ.get("DISPLAY", ":99"), "-screen", "0", "1600x900x24", "-ac", "+extension", "GLX", "+render", "-noreset"],
            xvfb_log,
        )
        service_outputs.append(output)
        wait_for_condition(
            lambda: display_is_ready(
                os.environ.get("DISPLAY", ":99"), arguments.command_timeout
            ),
            xvfb,
            arguments.startup_timeout,
            "xvfb_readiness",
            "Xvfb did not become ready",
            "Inspect xvfb.log and the DISPLAY value.",
        )
        window_manager, output = start_logged_process(
            ["fluxbox", "-display", os.environ.get("DISPLAY", ":99")],
            window_manager_log,
        )
        service_outputs.append(output)
        wait_for_condition(
            lambda: window_manager_is_ready(arguments.command_timeout),
            window_manager,
            arguments.startup_timeout,
            "window_manager_readiness",
            "Fluxbox did not publish a managed-window readiness property",
            "Inspect fluxbox.log and Fluxbox startup.",
        )
        vnc, output = start_logged_process(
            ["x11vnc", "-display", os.environ.get("DISPLAY", ":99"), "-forever", "-shared", "-nopw", "-rfbport", "5900"],
            vnc_log,
        )
        service_outputs.append(output)
        wait_for_internal_port(5900, vnc, arguments.startup_timeout, "vnc_readiness")
        novnc, output = start_logged_process(["websockify", "--web=/usr/share/novnc/", "6080", "127.0.0.1:5900"], novnc_log)
        service_outputs.append(output)
        wait_for_internal_port(6080, novnc, arguments.startup_timeout, "novnc_readiness")
        probe_novnc("http://127.0.0.1:6080/vnc.html", arguments.startup_timeout)
        renderer = renderer_name(arguments.command_timeout)

        simulator_environment = dict(os.environ)
        simulator_command = [
            str(simulator_root / "coppeliaSim.sh"),
            "-GsimCmd.autoStart=false",
            "-GpreferredSandboxLang=lua",
            f"-Gpython={os.environ.get('COPPELIASIM_PYTHON', '/usr/bin/python3')}",
            "-GzmqRemoteApi.rpcPort=23000",
        ]
        simulator, output = start_logged_process(simulator_command, simulator_log, simulator_environment)
        service_outputs.append(output)
        wait_for_condition(
            lambda: remote_api_is_ready(
                simulator_root, 23000, arguments.command_timeout
            ),
            simulator,
            arguments.startup_timeout,
            "remote_api_readiness",
            "CoppeliaSim ZeroMQ Remote API did not answer a protocol request",
            "Inspect simulator.log and the ZeroMQ Remote API service.",
        )
        window = locate_linux_window(
            simulator, arguments.window_timeout, arguments.command_timeout
        )
        run_bounded_probe(
            ["xdotool", "windowsize", str(window["window_id"]), "1500", "820"],
            arguments.command_timeout,
            "capture_window",
            "Inspect the owned CoppeliaSim X11 window.",
        )
        run_bounded_probe(
            ["xdotool", "windowmove", str(window["window_id"]), "40", "40"],
            arguments.command_timeout,
            "capture_window",
            "Inspect the owned CoppeliaSim X11 window.",
        )
        time.sleep(0.2)
        window = locate_linux_window(
            simulator, arguments.window_timeout, arguments.command_timeout
        )
        environment["display"] = {
            "xvfb": True,
            "window_manager": "fluxbox",
            "renderer": renderer,
            "software_rendering": os.environ.get("LIBGL_ALWAYS_SOFTWARE") == "1",
            "managed_window": True,
            "window": window,
        }
        host_url = os.environ.get("ROBORUN_LINUX_GUI_HOST_NOVNC_URL", "")
        observation = {
            "schema": "roborun.linux-gui.observation-ready.v1",
            "scenario": scenario,
            "scenario_id": scenario_id,
            "url": host_url,
            "window": window,
        }
        (case_directory / "observation-ready.json").write_text(json.dumps(observation, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        confirmation = wait_for_host_confirmation(case_directory / "observation-confirmed.json", arguments.host_observation_timeout)
        environment["novnc"] = {
            "internal_url": "http://127.0.0.1:6080/vnc.html",
            "url": host_url,
            "reachable_from_host": confirmation.get("url") == host_url,
            "observed_at": confirmation.get("observed_at"),
        }
        environment["captured_window"] = window
        environment_path.write_text(json.dumps(environment, indent=2, sort_keys=True) + "\n", encoding="utf-8")

        recorder = LinuxWindowRecorder(window, case_directory / "animation-frames", arguments.gif_fps, arguments.capture_timeout)
        recorder.start()
        runtime_command = container_runtime_command(
            scenario,
            scenario_id,
            ready_path,
            continue_path,
            scene_path,
            simulator_root,
            arguments.evidence_timeout,
        )
        runtime, output = start_logged_process(runtime_command, runtime_log)
        service_outputs.append(output)
        gui_evidence.wait_for_file(ready_path, runtime, arguments.scenario_timeout, "evidence_ready")
        ready = load_json(ready_path, "evidence_binding")
        if ready.get("scenario") != scenario or ready.get("scenario_id") != scenario_id:
            fail("evidence_binding", "Runtime evidence-ready record does not match this Linux scenario", "Keep each scenario in a fresh container and case directory.")
        screenshot_path = case_directory / f"{scenario_id}-generation-{ready.get('generation')}.png"
        try:
            frames = recorder.stop()
            capture_linux_window(window, screenshot_path, arguments.capture_timeout)
            create_linux_animation(
                scenario,
                scenario_id,
                case_directory,
                window,
                frames,
                screenshot_path,
                arguments.gif_fps,
                max(30.0, arguments.scenario_timeout),
            )
            for frame in frames:
                frame.unlink(missing_ok=True)
            recorder.directory.rmdir()
        except CheckError as error:
            capture_error = error
        finally:
            gui_evidence.confirm_capture(ready_path, continue_path)
        try:
            runtime_status = runtime.wait(timeout=30)
        except subprocess.TimeoutExpired:
            capture_error = capture_error or CheckError("runtime_exit", "Runtime did not exit after Linux capture confirmation", "Inspect runtime.log and evidence barrier cleanup.")
        if simulator.poll() is None:
            try:
                simulator.wait(timeout=30)
            except subprocess.TimeoutExpired:
                capture_error = capture_error or CheckError("gui_exit", "CoppeliaSim did not exit after quitSimulator", "Inspect lifecycle records and simulator.log.")
        environment["finished_at"] = datetime.now(timezone.utc).isoformat()
        environment_path.write_text(json.dumps(environment, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except CheckError as error:
        scenario_error = error
    except OSError as error:
        scenario_error = CheckError("container_process", str(error), "Inspect the declared Linux GUI evidence image dependencies.")
    finally:
        if recorder is not None:
            try:
                recorder.stop()
            except CheckError as error:
                capture_error = capture_error or error
        cleanup_succeeded = stop_owned_process_group(runtime) and cleanup_succeeded
        if runtime is not None and runtime_status is None:
            runtime_status = runtime.poll()
        cleanup_succeeded = stop_owned_process_group(simulator) and cleanup_succeeded
        cleanup_succeeded = stop_owned_process_group(novnc) and cleanup_succeeded
        cleanup_succeeded = stop_owned_process_group(vnc) and cleanup_succeeded
        cleanup_succeeded = stop_owned_process_group(window_manager) and cleanup_succeeded
        cleanup_succeeded = stop_owned_process_group(xvfb) and cleanup_succeeded
        for output in service_outputs:
            output.close()
        internal_ports = {
            "remote_api": not host_port_is_occupied(23000),
            "vnc": not host_port_is_occupied(5900),
            "novnc": not host_port_is_occupied(6080),
        }
        cleanup_succeeded = cleanup_succeeded and all(internal_ports.values())
        process_record = {
            "runtime_exit_status": runtime_status,
            "simulator_exit_status": simulator.poll() if simulator is not None else None,
            "display_exit_status": xvfb.poll() if xvfb is not None else None,
            "xvfb_exit_status": xvfb.poll() if xvfb is not None else None,
            "window_manager_exit_status": window_manager.poll()
            if window_manager is not None
            else None,
            "x11vnc_exit_status": vnc.poll() if vnc is not None else None,
            "novnc_exit_status": novnc.poll() if novnc is not None else None,
            "cleanup_succeeded": cleanup_succeeded,
            "internal_ports_released": internal_ports,
        }
        process_path.write_text(json.dumps(process_record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        retained_error = scenario_error or capture_error
        if retained_error is not None:
            gui_evidence.write_failure_record(case_directory, retained_error, process_record)
    if scenario_error is not None:
        raise scenario_error
    if capture_error is not None:
        raise capture_error
    expected_status = 0 if scenario == "normal" else 1
    if runtime_status != expected_status:
        fail("runtime_result", f"Runtime exited with status {runtime_status}, expected {expected_status}", "Inspect runtime.log.")


def add_host_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--docker", default=shutil.which("docker") or "/usr/local/bin/docker")
    parser.add_argument("--compatibility-mode", choices=("auto", "native", "rosetta"), default="auto")
    parser.add_argument("--remote-port", type=int, default=DEFAULT_REMOTE_PORT)
    parser.add_argument("--vnc-port", type=int, default=DEFAULT_VNC_PORT)
    parser.add_argument("--novnc-port", type=int, default=DEFAULT_NOVNC_PORT)
    parser.add_argument("--docker-timeout", type=float, default=60.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    preflight_parser = subparsers.add_parser("preflight", help="validate Docker, platform and host ports")
    add_host_options(preflight_parser)
    verify_parser = subparsers.add_parser("verify", help="verify one retained Linux GUI evidence scenario")
    verify_parser.add_argument("--scenario", choices=("normal", "estop"), required=True)
    verify_parser.add_argument("--case-dir", type=Path, required=True)
    compatibility_parser = subparsers.add_parser("compatibility", help="apply or inspect the Linux simulator compatibility profile")
    compatibility_parser.add_argument("--mode", choices=("native", "rosetta"), required=True)
    compatibility_parser.add_argument("--simulator-root", type=Path, required=True)
    container_parser = subparsers.add_parser("container-run", help=argparse.SUPPRESS)
    container_parser.add_argument("--case-dir", type=Path, required=True)
    container_parser.add_argument("--startup-timeout", type=float, default=60.0)
    container_parser.add_argument("--window-timeout", type=float, default=60.0)
    container_parser.add_argument("--host-observation-timeout", type=float, default=60.0)
    container_parser.add_argument("--scenario-timeout", type=float, default=120.0)
    container_parser.add_argument("--capture-timeout", type=float, default=20.0)
    container_parser.add_argument("--evidence-timeout", type=float, default=60.0)
    container_parser.add_argument("--gif-fps", type=float, default=4.0)
    container_parser.add_argument("--source-build-timeout", type=float, default=300.0)
    container_parser.add_argument("--command-timeout", type=float, default=5.0)
    run_parser = subparsers.add_parser("run", help="build the image and run real Linux GUI check")
    add_host_options(run_parser)
    run_parser.add_argument("--build-dir", type=Path, default=PROJECT_ROOT / ".build" / "linux-gui-evidence")
    run_parser.add_argument("--image", default=DEFAULT_IMAGE)
    run_parser.add_argument("--scenario", choices=("all", "normal", "estop"), default="all")
    run_parser.add_argument("--novnc-timeout", type=float, default=900.0)
    run_parser.add_argument("--scenario-timeout", type=float, default=180.0)
    run_parser.add_argument("--image-build-timeout", type=float, default=1200.0)
    run_parser.add_argument("--source-build-timeout", type=float, default=300.0)
    run_parser.add_argument("--command-timeout", type=float, default=5.0)
    run_parser.add_argument("--source-snapshot-timeout", type=float, default=120.0)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        if arguments.command == "preflight":
            environment = preflight_host(arguments)
            print(json.dumps(environment, sort_keys=True))
            print("RESULT status=passed phase=linux_gui_preflight")
        elif arguments.command == "verify":
            manifest = verify_linux_case(arguments.scenario, arguments.case_dir)
            print(f"RESULT status=passed phase=linux_gui_case_verification scenario={arguments.scenario} generation={manifest['generation']}")
        elif arguments.command == "compatibility":
            record = apply_compatibility(arguments.mode, arguments.simulator_root)
            print(json.dumps(record, sort_keys=True))
            print("RESULT status=passed phase=linux_gui_compatibility")
        elif arguments.command == "container-run":
            container_run(arguments)
            print(f"RESULT status=passed phase=linux_gui_container_scenario scenario={os.environ.get('ROBORUN_LINUX_GUI_SCENARIO')}")
        elif arguments.command == "run":
            evidence = run_check(arguments)
            print(f"RESULT status=passed phase=linux_gui_machine_check scenario={arguments.scenario} evidence={quote(str(evidence))}")
        return 0
    except CheckError as error:
        gui_evidence.print_failure(error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
