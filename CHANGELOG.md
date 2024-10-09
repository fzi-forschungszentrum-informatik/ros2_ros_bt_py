# Changelog

History of changes and bugfixes for ros2_ros_bt_py

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [unreleased]

### Added

- Tests for NodeConfig class
- Tree storage path is now a launch argument
- Action results can now be processed when using ABC Actions
- Documentation on how to use custom node classes with the library

### Changed

- Fixed up documentation from ROS 1 version
- Updated Web-GUI version to 2.0.5
- Subtree manager is now its own class and not inside the debug manager
- Updated NodeDiagnostics
- Library can now be installed with symlink install
- Module List is now generated in a separate function
- WebGUI is now launched on default
- Service and Action Nodes now directly expose Request/Response and Goal/Result/Feedback fields

### Fixed

- Fixed missing attributes in the Action node.
- Fixed high level exception catching masking errors.
- Fixed race condition when setting the tree state.
- Fixed package manager to be able to handle more than one package
- Fixed python version to python3
- Fixed Action Node and internal state machine
- Fixed Wait Node to reset before shutdown

## [v0.0.1]

[unreleased]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/compare/v0.0.1...dev
[v0.0.1]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/releases/tag/v0.0.1

<!---
## [vx.x.x] - YYYY-MM-DD

### Added

- Put all Additions to the repository in here

### Changed

- Put all Changes in existing functionality here

### Deprecated

- Put all soon-to-be removed features here

### Removed

- Put all removed features here

### Fixed

- Put bugfixes here

[vx.x.x]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/compare/OLDTAG...NEWTAG
-->
