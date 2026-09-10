# Fixed Third-Party Dependencies

RoboRun keeps the following source snapshots in this directory. CMake uses these local copies and
does not fetch them during configure, build, or test.

| Dependency | Version | Canonical upstream | Pinned archive SHA-256 | Included path | Purpose | Distribution role | License |
| --- | --- | --- | --- | --- |
| nlohmann/json | 3.12.0 | https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz | `42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa` | `third_party/nlohmann_json/` | Private JSON configuration implementation | Source archive only; not installed or exported | MIT, `third_party/nlohmann_json/LICENSE.MIT` |
| GoogleTest | 1.17.0 | https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz | `65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c` | `third_party/googletest/` | Test-only framework | Source archive only; not installed or exported | BSD-3-Clause, `third_party/googletest/LICENSE` |
