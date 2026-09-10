#!/usr/bin/env python3
"""Check first-party C++ formatting and basic CMake whitespace rules."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


CPP_SUFFIXES = {".cpp", ".h"}
CPP_ROOTS = {"ros2", "src", "tests"}
CMAKE_ROOTS = {"cmake", "ros2", "scripts", "src", "tests"}


def first_party_files(repository: Path) -> tuple[list[Path], list[Path]]:
    cpp_files: list[Path] = []
    cmake_files: list[Path] = []
    for path in repository.rglob("*"):
        relative = path.relative_to(repository)
        if not path.is_file():
            continue
        root = relative.parts[0]
        if path.suffix in CPP_SUFFIXES and root in CPP_ROOTS:
            cpp_files.append(path)
        if (path.name == "CMakeLists.txt" or path.suffix == ".cmake") and (
            len(relative.parts) == 1 or root in CMAKE_ROOTS
        ):
            cmake_files.append(path)
    return sorted(cpp_files), sorted(cmake_files)


def check_cmake_whitespace(files: list[Path]) -> int:
    invalid: list[str] = []
    for path in files:
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            if "\t" in line or line.rstrip() != line:
                invalid.append(f"{path}:{line_number}")
    if invalid:
        print("CMake files contain tabs or trailing whitespace:", file=sys.stderr)
        print("\n".join(invalid), file=sys.stderr)
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, default=Path.cwd())
    parser.add_argument("--clang-format", default="clang-format")
    arguments = parser.parse_args()

    repository = arguments.repository.resolve()
    cpp_files, cmake_files = first_party_files(repository)
    if check_cmake_whitespace(cmake_files) != 0:
        return 1
    if not cpp_files:
        return 0
    result = subprocess.run(
        [arguments.clang_format, "--dry-run", "--Werror", *map(str, cpp_files)],
        check=False,
    )
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
