# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- Fix launch file sets invalid parameter values.


## [0.4.0] - 2025-05-22

### Added
- Added tests for action node
- Add git cliff for changelog management

### Changed
- Release 0.4.0
- Merge pull request #144 from mdeitersen/add_action_tests
- Changed theme to be inline with ros2 documentation
- Updated package.xml and CMakeLists.txt
- Corrected import path
- Merge pull request #170 from Lukas1407/docu-rework
- Changed theme to be inline with ros2 documentation
- Changle cliff.toml
- Changle cliff.toml
- Merge pull request #177 from fzi-forschungszentrum-informatik/update-web-gui-to-4.0.1
- Update Web-GUI to 4.0.1
- Merge pull request #175 from fzi-forschungszentrum-informatik/update-web-gui-to-4.0.0
- Update Web-GUI to 4.0.0
- Merge pull request #171 from Doomse/tree-message-rework
- Update tests to new tree message types
- Always collect subtree messages and toggle publishing in manager.
- Properly set tree ids and clean up outdated subtree
- Clear tree data on disable
- Manually enable data publish, data is published with every tick
- Include subtree information in tree topics
- Change ppublishing to produce more consistent results and overwrite stale data
- Publish Wiring Data if tree is run with 'Tick Once'
- Revert to explicitly specifying "serialized" on message fields
- Implement tree state publishing
- Update tree manager part1
- Update imports for state constants
- Update service definitions
- Rework message types

### Fixed
- Fix import error
- Fix issues with action fail on available
- Fix mock usage
- Fix seconds_running calculation error in action
- Fix changelog ci job
- Fix changelog ci job
- Fix cleanup of outdated subtrees
- Fix initially apparent errors (from message class rework)

### Removed
- Remove commented out code
- Remove data publish on unwire, since that dumps all data (tree is shutdown)


## [0.3.0] - 2025-02-26

### Added
- Add custom types
- Add typing hints for builtin

### Changed
- Merge pull request #165 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.3.1
- Update Web-GUI to release/3.3.1
- Merge pull request #164 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.3.0
- Merge branch 'main' into update-web-gui-to-release/3.3.0
- Update Web-GUI to release/3.3.0
- Merge pull request #163 from fzi-forschungszentrum-informatik/fix_memory_subscriber
- Merge pull request #161 from Doomse/node-meta-fix
- Integrate abc into NodeMeta and Node classes
- Reenable Node metaclass and fix docstring generator
- Merge pull request #162 from Doomse/enum-values
- Update tests for EnumFields node
- Apply TopicType to EnumFields
- Merge pull request #159 from Doomse/type-hints
- Update ConstFields Service
- Integrate TypeWrapper with compatibility conversions
- Implement generic type wrapper
- Update get_message_fields to provide recursive type information
- Allow for node options to give type hints
- Switch to best_effort for tree publishing.

### Fixed
- Fixed TopicSubscriber node behavior
- Fix message constant fields service including tests

### Removed
- Remove non-functional enum node


## [0.2.0] - 2025-01-23

### Added
- Add handler to allow the loading of old trees.
- Add default values to Ros Message types
- Add action name discovery
- Add Ros Service types
- Added missing )
- Added showcase gif
- Added line between icons and header

### Changed
- Merge pull request #154 from fzi-forschungszentrum-informatik/type-system-web-update
- Merge pull request #156 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.2.2
- Merge branch 'type-system-web-update' into update-web-gui-to-release/3.2.2
- Merge pull request #151 from Doomse/type_system
- Restructure MessageType publlishing
- Keep maths types in helpers.py for compatibility
- Implement Ros Topic types
- Implement Ros Action types and baseclasses
- Publish names and types of existing topics and services. Actions still missing
- Move math types to new location
- Implement FilePath type
- Merge pull request #143 from fzi-forschungszentrum-informatik/rework_landing_page
- Updated Changelog
- Updated gif
- Merge pull request #158 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.2.3
- Update CI workflow.
- Update Web-GUI to release/3.2.3
- Update Web-GUI to release/3.1.0

### Fixed
- Fix invalid default value.
- Fix test_wait_for_service_input
- Fix test_wait_for_service
- Fix test_service_input
- Fix test_service
- Fix test_message_to_fields
- Fix test_message_from_dict_const
- Fix test_message_from_dict
- Fix test_fields_to_message
- Fixed up README

### Removed
- Remove empty action.
- Remove checks that are now redundant


## [0.1.1] - 2024-11-04

### Added
- Added documentation fix to changelog
- Added mock imports for autodoc
- Added tree storage path documentation
- Add codecov config.
- Added manual documentation build to doc dir

### Changed
- Merge pull request #146 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.0.0
- Update Web-GUI to release/3.0.0
- Merge pull request #139 from fzi-forschungszentrum-informatik/fix_documentation_build
- Merge pull request #141 from fzi-forschungszentrum-informatik/doc_tree_storage_path
- Merge pull request #126 from sea-bass/distros-in-ci
- Merge branch 'main' into distros-in-ci
- Bumped doc version

### Fixed
- Fixed module path for autodoc
- Fix documentation makefile back to tabs.

### Removed
- Removed faulty header
- Removed shoving ref which was not defined
- Removed some !-marks
- Remove docu build from README


## [0.1.0] - 2024-10-28

### Added
- Added more complex tutorials
- Added ROS interfacing
- Adding subtree section
- Added custom launch documentation
- Added clarification on GetAttr usage
- Added time module to changelog
- Added time module and TimeNow Node
- Add set_state function to ensure valid state tran.
- Added subtree_manager to overwritten inits
- Added new module list function
- Add exception for jazzy on workflow after install script.
- Add jazzy to ici config.
- Added tree_storage_path as launch argument
- Add tests.
- Added comments
- Added missing copyright text for test classes
- Added subtree manager to all nodes
- Added new SubtreeInfo Topic
- Added extension tests
- Added to examples
- Added internal tests
- Added Init tests for NodeConfig
- Added tests for OptionRef class
- Add Jazzy and Rolling to CI
- Add Jazzy / Ubuntu 24.04 to Git issue template

### Changed
- Merge pull request #131 from fzi-forschungszentrum-informatik/dev
- Made RandomIntInput consistent with RandomInt
- Updated RandomInt Node description
- Adjusted RandomInt to make more sense
- Reworked getting started and added tutorials
- Reworked header levels
- Renamed chapter to fit better
- Updated landing page
- Add missing attributes in topic.py
- Switch to MultiThreadedExecutor.
- Cleanly shut down tree node.
- Updated CHANGELOG
- Adapted tests for Service nodes
- Show action goal, result and feedback fields in Action nodes
- Directly show service request and response fields in Service nodes.
- Enabled default value on launch
- Merge pull request #95 from fzi-forschungszentrum-informatik/fb_fix_module_list
- Merge pull request #78 from fzi-forschungszentrum-informatik/fb_remove_debug_code
- Merge branch 'dev' into fb_remove_debug_code
- Overwrite output method
- Result can now be processed when using ABC Actions
- Ignore parameters module in test_import.py
- Reset wait nodes before shutdown.
- Merge branch 'fb_remove_debug_code' into update-web-gui-to-release/2.0.5
- Deleted duplicated conditional statement
- Replaced NodeDiagnostics message with DiagnosticStatus message
- Replaced  SetExecutionMode.srv for setting collect_node_diagnostics in DebugManager and publish_subtrees in SubtreeManager  with two seperate services
- Moved subtree manager from debug manager to another class
- Merge branch 'fix_create_subtree_from_nodes' into update-web-gui-to-release/2.0.4
- Okay this seems to work, idk why
- Changed order inside asserts
- Update Web-GUI to release/2.0.3
- Updated TOC in all CONTRIBUTING.md files
- Update Web-GUI to release/2.0.4
- Update Web-GUI to release/2.0.5
- Update industrial_ci.yml
- Update industrial_ci.yml
- Update BUG-REPORT.yml
- Packages list only ever contained one item

### Fixed
- Fixed package name
- Fix bug: unable remove Action node with initial default values
- Fix invalid usage of self.tree.state
- Fix undeclared state variable in action node.
- Fix missing check for feedback in action node.
- Fix self.name is None error.
- Fix service call mock assert.
- Fix style issues.
- Fix error with subtree initializations.
- Fix symlink install parameter_library bug.
- Fixed optional options to be a list
- Fixed typo
- Fixed assert
- Fix cancellation of goal.
- Fix variable names in action node.
- Fix action state machine.
- Fix action broken status after succeeding.
- Fixed imports in docs: creating_node_classes.rst
- Fixed Python version

### Removed
- Remove _setting_up variable.
- Remove unneeded passthrough parameter.
- Remove Client import in action.py
- Remove verbose logging.
- Remove top-level exception catching.
- Remove unnecessary logging.
- Remove unnecessary logging messages.
- Remove ament_flake8 as it conflicts with black.
- Remove print statements.
- Removed NodeDiagnostics.msg from CmakeLists
- Removed logger information which was temporarily added for debugging purposes
- Removed debugging functionalities from DebugManager and DebugInfo
- Removed modify_breakpoints
- Remove ament_auto_lint from ros_bt_py_web_gui


## [0.0.1] - 2024-02-22

### Added
- Added basic template for CHANGELOG
- Add documentation link to README.
- Add missing theme dependency/
- Add missing dependency
- Add sphinx doc build job.
- Add Github Actions CI
- Added tests for debug_manager to test subtree state
- Added test for ros_helpers.py
- Added test cases for helpers.py
- Add codecov github action
- Add coverage reporting.
- Add github workflow and issue templates
- Add msg type to GetMessageFields message.
- Add rules to make rolling optional.
- Add non-ros nodes to coverage config.
- Add tests for param nodes
- Add tests for wait_for_service nodes
- Add tests for Service node
- Add tests for ServiceInput
- Add ros_node reference to Node instances
- Add note about generate_param_lib and symlink_install
- Add getting started section
- Add sphinx based documentation.
- Add README.md
- Add url printer for web gui
- Add web_gui to ros_bt_py launch file.
- Add web_gui and launch files
- Add upstream workspace for generate_parameter_library
- Add upstream workspace for generate_parameter_library
- Add .coverage to gitignore
- Add tests for package_manager.py
- Add internal Gitlab CI to packages
- Add ament_lint linters to pre-commit.

### Changed
- Merge pull request #59 from fzi-forschungszentrum-informatik/dev
- Update version numbers to match changelog.
- Update Web-GUI to release/2.0.2
- Merge branch 'main' into dev
- Perform unwiring first before removing node
- Set all node states to shutdown and tree state to idle when saving a tree
- Merge pull request #47 from mdeitersen/add_helper_tests
- Merge branch 'main' into add_helper_tests
- Only upload single test report and tag code coverage
- Update Readme.
- Update industrial_ci.yml
- Update industrial_ci.yml
- Update industrial_ci.yml
- Update industrial_ci.yml
- Update industrial_ci.yml
- Merge branch 'prepare_github_release' into 'main'
- Prepare GitHub Release.
- Implement proper type completion for the JSON Editor.
- Update webgui JSON input.
- Merge branch 'main' into fb_web_gui_json_editor_type_completion
- Merge branch 'main' into fix_subtree_constructor
- Merge branch 'fix_ci_rolling_allow_to_fail' into 'main'
- Merge branch 'fb_load_save' into 'main'
- Implement save_to_path fuction
- Merge branch 'fix_installation_code_snippets' into 'main'
- Use double backticks for referencing code snipptets in RST
- Merge branch 'fix_destruction_error' into 'main'
- Merge branch 'port_ros_nodes' into 'main'
- Merge branch 'main' into 'port_ros_nodes'
- Port file.py nodes
- Port throttle.py nodes.
- Port subtree.py
- Port action.py node
- Remote test case that failes due to changes in rolling
- Port topic.py
- Port enum.py and add tests
- Update pytest config.
- Port message_from_dict nodes
- Port message_converters.py
- Port RosParamInput and RosParamOptionDefaultInput
- Port RosParamOption
- Change code coverage report file paths
- Use relative paths in coberatura CI report.
- Enable reports for test coverage
- Update ci config
- Port service nodes
- Merge branch 'update_readme' into 'main'
- Merge branch 'fb_add_basic_nodes' into 'main'
- Import node modules in __init__.py
- Adapted to new folder structure for src files
- Porting of ROS independent nodes
- Merge branch 'add_documentation' into 'main'
- Print web gui URL during startup.
- Merge branch 'add_web_gui' into 'main'
- Switch to master ci_scripts branch.
- Switch to fix_coverage_eos2 CI branch.
- Update web_gui to use new interfaces.
- Merge branch 'port_tree_manager' into 'main'
- Configure pytest to respect generated code.
- Switch to generate_parameter_library
- Make tree node executable.
- Port tree_node.py
- Port tree_manager.py
- Port package_manager.py
- Port package_manager.py
- Try importing each submodule from ros_bt_py
- Try importing each submodule from ros_bt_py
- Port node class and related code
- Port debug_manager.py
- Port node.py
- Port node_config.py
- Port node_data.py
- Port helper functions
- Run linters as part of pre-commit not colcon test.
- Ignore invalid warning from flake8.
- Port ros_helpers class.
- Port helpers.py
- Port exceptions.py
- Initial commit.

### Fixed
- Fix invalid subtree creation while saving a tree.
- Fix missing parameters in nodes.
- Fix github variables error
- Fix github variables error
- Fix github variable problem.
- Fix missing package.xml dependencies.
- Fix ci configuration.
- Fix inconsistent style.
- Fix missing parameter in PackageManager tests
- Fixed code snippets
- Fix subtree constructor
- Fix service destruction error.
- Fix topic destruction_error
- Fix message converters drag and drop.
- Fix service call and message autocompletion
- Fix error with invalid ros_node reference
- Fix node imports and tree_manager service calls.
- Fix gitlab-ci.yml
- Fix author in latex report
- Fix CI errors for web_gui.
- Fix pipeline test reporting.
- Fix pipeline test reporting.
- Fix service callback signatures in package_manager.
- Fix parameter loading and add launchfile.
- Fix type errors.
- Fix pyproject.toml
- Fix toml error in CI.
- Fix node import
- Fix code style issues.
- Fix code style issues.
- Fix missing import in interfaces CMakeLists.txt
- Fix CI configuration and add pre-commit to CI.
- Fix pipeline config.

### Removed
- Remove dependencies that are not covered by rosdep
- Removed fix_yaml
- Remove usage of an invalid constructor for the subtree class.
- Remove old warning from README
- Remove unused imports
- Remove test parameter that is ros2 version dependent.


[unreleased]: https://github.com///compare/v0.4.0..HEAD
[0.4.0]: https://github.com///compare/v0.3.0..v0.4.0
[0.3.0]: https://github.com///compare/v0.2.0..v0.3.0
[0.2.0]: https://github.com///compare/v0.1.1..v0.2.0
[0.1.1]: https://github.com///compare/v0.1.0..v0.1.1
[0.1.0]: https://github.com///compare/v0.0.1..v0.1.0

<!-- generated by git-cliff -->
