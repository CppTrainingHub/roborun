#!/usr/bin/env python3
"""Create deterministic RoboRun source archives from one committed Git tree."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import io
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import zipfile


DEPENDENCY_RECORDS = ("THIRD_PARTY_NOTICES.md", "third_party/DEPENDENCIES.md")
DEPENDENCY_ARCHIVE_SUFFIXES = (".tar.gz", ".tar.xz", ".zip")


def run_git(repository: Path, *arguments: str) -> bytes:
    return subprocess.run(
        ["git", "-C", str(repository), *arguments],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ).stdout


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def project_version(repository: Path, commit: str) -> str:
    cmake_lists = run_git(repository, "show", f"{commit}:CMakeLists.txt").decode("utf-8")
    match = re.search(r"project\(RoboRun\s+VERSION\s+([^\s)]+)", cmake_lists)
    if match is None:
        raise RuntimeError("CMakeLists.txt does not declare project(RoboRun VERSION ...).")
    return match.group(1)


def validate_dependency_inventory(repository: Path, commit: str,
                                  member_names: set[str]) -> None:
    inventory_directories: set[str] = set()
    for record in DEPENDENCY_RECORDS:
        if record not in member_names:
            raise RuntimeError(f"Source archive is missing dependency record: {record}")
        contents = run_git(repository, "show", f"{commit}:{record}").decode("utf-8")
        for distributed_path in re.findall(r"`(third_party/[^`]+)`", contents):
            if distributed_path.endswith("/"):
                inventory_directories.add(distributed_path.rstrip("/"))
                if not any(name.startswith(distributed_path) for name in member_names):
                    raise RuntimeError(
                        f"Dependency inventory path is absent from the archive: {distributed_path}"
                    )
            elif distributed_path not in member_names:
                raise RuntimeError(
                    f"Dependency license is absent from the archive: {distributed_path}"
                )

    bundled_directories = {
        "/".join(name.split("/")[:2])
        for name in member_names
        if name.startswith("third_party/") and len(name.split("/")) > 2
    }
    unlisted_directories = sorted(bundled_directories - inventory_directories)
    if unlisted_directories:
        raise RuntimeError(
            "Source archive contains dependencies missing from the inventory: "
            + ", ".join(unlisted_directories)
        )

    duplicate_archives = sorted(
        name
        for name in member_names
        if name.startswith("third_party/") and name.endswith(DEPENDENCY_ARCHIVE_SUFFIXES)
    )
    if duplicate_archives:
        raise RuntimeError(
            "Source archive contains duplicate dependency downloads: "
            + ", ".join(duplicate_archives)
        )


def write_archives(repository: Path, commit: str, output_directory: Path,
                   version: str) -> tuple[Path, Path, Path]:
    prefix = f"RoboRun-{version}"
    archive = run_git(repository, "archive", "--format=tar", f"--prefix={prefix}/", commit)
    source = tarfile.open(fileobj=io.BytesIO(archive), mode="r:")
    members = source.getmembers()
    members.sort(key=lambda member: member.name)
    member_names = {
        member.name[len(prefix) + 1:]
        for member in members
        if member.isfile() and member.name.startswith(f"{prefix}/")
    }
    validate_dependency_inventory(repository, commit, member_names)

    tar_path = output_directory / f"{prefix}.tar.gz"
    with tar_path.open("wb") as destination:
        with gzip.GzipFile(filename="", mode="wb", fileobj=destination, mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w:") as tar_output:
                for member in members:
                    member.mtime = 0
                    member.uid = 0
                    member.gid = 0
                    member.uname = ""
                    member.gname = ""
                    data = source.extractfile(member) if member.isfile() else None
                    tar_output.addfile(member, data)

    zip_path = output_directory / f"{prefix}.zip"
    with zipfile.ZipFile(zip_path, mode="w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as zip_output:
        for member in members:
            entry = zipfile.ZipInfo(member.name, date_time=(1980, 1, 1, 0, 0, 0))
            entry.create_system = 3
            entry.external_attr = (member.mode & 0xFFFF) << 16
            if member.isdir():
                entry.external_attr |= 0x10
                zip_output.writestr(entry, b"")
            elif member.isfile():
                data = source.extractfile(member)
                if data is None:
                    raise RuntimeError(f"Could not read archive member: {member.name}")
                zip_output.writestr(entry, data.read(), compress_type=zipfile.ZIP_DEFLATED,
                                    compresslevel=9)
            else:
                raise RuntimeError(f"Unsupported Git archive member: {member.name}")
    source.close()

    manifest_path = output_directory / f"{prefix}.manifest.txt"
    manifest_entries: list[str] = []
    for member in members:
        if member.isdir():
            manifest_entries.append(f"directory  {member.name}")
            continue
        data = run_git(repository, "show", f"{commit}:{member.name[len(prefix) + 1:]}")
        manifest_entries.append(f"{hashlib.sha256(data).hexdigest()}  {member.name}")
    manifest_path.write_text("\n".join(sorted(manifest_entries)) + "\n", encoding="utf-8")
    return tar_path, zip_path, manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, default=Path.cwd())
    parser.add_argument("--commit", required=True, help="Full or resolvable Git commit to archive")
    parser.add_argument("--output-dir", required=True, type=Path)
    arguments = parser.parse_args()

    repository = arguments.repository.resolve()
    commit = run_git(repository, "rev-parse", "--verify", f"{arguments.commit}^{{commit}}").decode().strip()
    output_directory = arguments.output_dir.resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    version = project_version(repository, commit)
    tar_path, zip_path, manifest_path = write_archives(repository, commit, output_directory, version)
    prefix = f"RoboRun-{version}"
    (output_directory / f"{prefix}.commit.txt").write_text(f"{commit}\n", encoding="utf-8")
    (output_directory / f"{prefix}.sha256").write_text(
        f"{sha256_file(tar_path)}  {tar_path.name}\n{sha256_file(zip_path)}  {zip_path.name}\n",
        encoding="utf-8",
    )
    print(f"created {tar_path}")
    print(f"created {zip_path}")
    print(f"created {manifest_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
