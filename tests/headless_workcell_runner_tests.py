#!/usr/bin/env python3
"""Preflight behavior tests for the CoppeliaSim headless check runner."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNNER = PROJECT_ROOT / "scripts" / "run_coppeliasim_workcell.sh"


class HeadlessWorkcellRunnerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.incomplete_app = self.root / "coppeliaSim.app"

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def run_runner(
        self, python: Path | None
    ) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["COPPELIASIM_APP"] = str(self.incomplete_app)
        if python is None:
            environment.pop("COPPELIASIM_PYTHON", None)
        else:
            environment["COPPELIASIM_PYTHON"] = str(python)
        return subprocess.run(
            [str(RUNNER)],
            cwd=PROJECT_ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )

    def make_python(self, exit_status: int) -> Path:
        executable = self.root / f"python-status-{exit_status}"
        executable.write_text(f"#!/bin/sh\nexit {exit_status}\n", encoding="utf-8")
        executable.chmod(0o755)
        return executable

    def test_requires_an_explicit_python_interpreter(self) -> None:
        result = self.run_runner(None)

        self.assertEqual(result.returncode, 2)
        self.assertIn("COPPELIASIM_PYTHON", result.stderr)

    def test_rejects_a_non_executable_python_interpreter(self) -> None:
        result = self.run_runner(self.root / "missing-python")

        self.assertEqual(result.returncode, 2)
        self.assertIn("not an executable Python interpreter", result.stderr)

    def test_rejects_a_python_environment_without_remote_api_modules(self) -> None:
        result = self.run_runner(self.make_python(exit_status=1))

        self.assertEqual(result.returncode, 2)
        self.assertIn("pyzmq", result.stderr)
        self.assertIn("cbor2", result.stderr)


if __name__ == "__main__":
    unittest.main()
