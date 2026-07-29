# AGENTS.md — ros2_ros_bt_py

## Project Overview

ROS 2 Behavior Tree library (`ros_bt_py`) with a ReactJS web GUI. Alternative to BehaviorTree.cpp, SMACH, FlexBE.

**Packages:**
- `ros_bt_py` — Main Python package (Behavior Tree engine, node classes, tree manager)
- `ros_bt_py_interfaces` — ROS 2 message/service interfaces
- `ros_bt_py_web_gui` — Web interface (ReactJS-based)

## Build & Test

**Prerequisites:** ROS 2 (humble, jazzy, kilted/rolling). Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

**Build:**
```bash
colcon build --packages-up-to ros_bt_py
source install/setup.sh
```

**Test (single package):**
```bash
colcon test --packages-select ros_bt_py
colcon test-result --all
```

**Coverage:** Enabled via `ENABLE_COVERAGE_TESTING=true`. Results uploaded to Codecov.

**Lint & Format:** Uses pre-commit. Run all checks:
```bash
pre-commit run --all
```

**Order:** `pre-commit` → `build` → `test`

## Architecture

**Core modules** (`ros_bt_py/ros_bt_py/`):
- `tree_node.py` — ROS 2 node entrypoint (console_script: `tree_node`)
- `tree_manager.py` — Tree lifecycle management (load, tick, save)
- `node.py` — Base node class and execution logic
- `node_config.py` — Node configuration system
- `node_data.py` — Dataflow and type-safe data ports
- `subtree_manager.py` — Nested tree handling
- `helpers.py` — Serialization, tree helpers
- `custom_types.py` — Type definitions for dataflow

**Node libraries:**
- `nodes/` — Core node implementations (control flow, dataflow)
- `ros_nodes/` — ROS-specific nodes (Actions, Services, Publishers, Subscribers)

**Entry points:**
- `ros2 launch ros_bt_py ros_bt_py.launch.py enable_web_interface:=True`
- Web GUI: `http://localhost:8085/index.html`

## Code Style

**Python:** ROS 2 ament_lint standards via pre-commit:
- `black` (py36+)
- `flake8`
- `ament_copyright` (BSD-3-Clause header required)
- `ament_xmllint` (for XML files)
- `ament_lint_cmake` (for CMakeLists.txt)

**Pre-commit config:** `.pre-commit-config.yaml`

**Important:** All new files must include BSD-3-Clause copyright header. Use `ament_copyright` to check.

## Testing

**Framework:** `pytest` with `pytest-cov`

**Config:** `pyproject.toml` — `asyncio_mode=auto`, `import-mode=importlib`

**Test location:** `ros_bt_py/tests/`

**Coverage omit:** `/usr*`, `/opt*`, `*test/*`, `*tests/*`, `*.local/*`, `*setup.py`

**Integration tests:** Use `launch_testing`, `launch_pytest`, `launch_testing_ros`. May require `domain_coordinator` for isolated ROS 2 domains.

## CI/CD

**GitHub Actions:** `.github/workflows/build_test.yml`
- pre-commit (kilted, jazzy)
- industrial_ci (humble, jazzy, kilted)
- Coverage upload to Codecov
- Test result publishing

**GitLab CI:** `.gitlab-ci.yml` — Mirrors GitHub with additional doc build and deployment.

**Supported ROS 2 distros:** humble, jazzy, rolling/kilted

## Development Workflow

**Branch:** `main` is active development (may be unstable). Use tagged releases for stable versions.

**Commits:** Standard format. GPG signing recommended.

**PRs:** Run pre-commit locally before pushing. CI validates on push/PR.

**Documentation:** Sphinx in `ros_bt_py/doc/source/`. Build:
```bash
sphinx-build -b html ros_bt_py/doc/source doc_public
```

## Quirks & Gotchas

**Parameter library:** Uses `generate_parameter_library_py` to generate Python parameter module from `config/tree_node_parameters.yaml`. Regenerates during `setup.py` execution.

**Vendor directory:** `ros_bt_py/vendor/` contains vendored dependencies (e.g., `result` library). Exclude from copyright checks.

**Tree files:** YAML format in `trees/`. Migration script available: `ros2 run ros_bt_py migrate_trees`

**Web GUI standalone:**
```bash
ros2 launch ros_bt_py_web_gui ros_bt_py_web_gui.launch.py web_server_port:=8085 web_server_address:=0.0.0.0
```

**Coverage path mapping:** `pyproject.toml` `[tool.coverage.paths]` maps build paths for CI coverage reporting.

## References

- **README.md** — Installation, launch arguments, usage
- **CONTRIBUTING.md** — Bug reports, enhancement suggestions, styleguide
- **Documentation:** https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/
