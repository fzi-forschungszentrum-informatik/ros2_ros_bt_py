# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Add pre-commit-ci job and rename workflow. by @Oberacda
- Add remote repo info in cliff.toml by @Oberacda

### Changed
- Update status badge. by @Oberacda
- Use flake8 instead of ament_flake8 by @Oberacda

### Fixed
- Fix TopicSubscriber durability policy by @RobertWilbrandt
- Fix formatting. by @Oberacda
- Put merges in seperate changelog section by @Oberacda
- Exclude changelog changes from changelog. by @Oberacda
- Fix launch file sets invalid parameter values. by @Oberacda

### Merged
- Merge pull request #184 from RobertWilbrandt/topic_durability_policy by @Oberacda in [#184](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/184)
- Merge pull request #180 from fzi-forschungszentrum-informatik/add_pre-commit_ci by @Oberacda in [#180](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/180)
- Merge pull request #179 from fzi-forschungszentrum-informatik/fix_exclude_changelog_changes by @Oberacda in [#179](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/179)
- Merge branch 'main' into fix_exclude_changelog_changes by @Oberacda
- Merge pull request #178 from fzi-forschungszentrum-informatik/fix_launch_file by @Oberacda in [#178](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/178)

### Removed
- Remove no-main commit check. by @Oberacda

## New Contributors
* @RobertWilbrandt made their first contribution

## [0.4.0] - 2025-05-22

### Added
- Added tests for action node by @mdeitersen
- Add git cliff for changelog management by @Oberacda

### Changed
- Release 0.4.0 by @Oberacda
- Changed theme to be inline with ros2 documentation
- Updated package.xml and CMakeLists.txt by @mdeitersen
- Corrected import path by @mdeitersen
- Changed theme to be inline with ros2 documentation
- Changle cliff.toml by @Oberacda
- Changle cliff.toml by @Oberacda
- Update Web-GUI to 4.0.1 by @Oberacda
- Update Web-GUI to 4.0.0 by @Oberacda
- Update tests to new tree message types by @Doomse
- Always collect subtree messages and toggle publishing in manager. by @Doomse
- Properly set tree ids and clean up outdated subtree by @Doomse
- Clear tree data on disable by @Doomse
- Manually enable data publish, data is published with every tick by @Doomse
- Include subtree information in tree topics by @Doomse
- Change ppublishing to produce more consistent results and overwrite stale data by @Doomse
- Publish Wiring Data if tree is run with 'Tick Once' by @Doomse
- Revert to explicitly specifying "serialized" on message fields by @Doomse
- Implement tree state publishing by @Doomse
- Update tree manager part1 by @Doomse
- Update imports for state constants by @Doomse
- Update service definitions by @Doomse
- Rework message types by @Doomse

### Fixed
- Fix import error by @Oberacda
- Fix issues with action fail on available by @Oberacda
- Fix mock usage by @mdeitersen
- Fix seconds_running calculation error in action by @mdeitersen
- Fix changelog ci job by @Oberacda
- Fix changelog ci job by @Oberacda
- Fix cleanup of outdated subtrees by @Doomse
- Fix initially apparent errors (from message class rework) by @Doomse

### Merged
- Merge pull request #144 from mdeitersen/add_action_tests by @Oberacda in [#144](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/144)
- Merge pull request #170 from Lukas1407/docu-rework by @Oberacda in [#170](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/170)
- Merge pull request #177 from fzi-forschungszentrum-informatik/update-web-gui-to-4.0.1 by @Oberacda in [#177](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/177)
- Merge pull request #175 from fzi-forschungszentrum-informatik/update-web-gui-to-4.0.0 by @Oberacda in [#175](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/175)
- Merge pull request #171 from Doomse/tree-message-rework by @Oberacda in [#171](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/171)

### Removed
- Remove commented out code by @mdeitersen
- Remove data publish on unwire, since that dumps all data (tree is shutdown) by @Doomse

## New Contributors
* @github-actions[bot] made their first contribution

## [0.3.0] - 2025-02-26

### Added
- Add custom types by @Doomse
- Add typing hints for builtin by @Doomse

### Changed
- Update Web-GUI to release/3.3.1 by @Oberacda
- Update Web-GUI to release/3.3.0 by @Oberacda
- Integrate abc into NodeMeta and Node classes by @Doomse
- Reenable Node metaclass and fix docstring generator by @Doomse
- Update tests for EnumFields node by @Doomse
- Apply TopicType to EnumFields by @Doomse
- Update ConstFields Service by @Doomse
- Integrate TypeWrapper with compatibility conversions by @Doomse
- Implement generic type wrapper by @Doomse
- Update get_message_fields to provide recursive type information by @Doomse
- Allow for node options to give type hints by @Doomse
- Switch to best_effort for tree publishing. by @Oberacda

### Fixed
- Fixed TopicSubscriber node behavior by @Oberacda
- Fix message constant fields service including tests by @Doomse

### Merged
- Merge pull request #165 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.3.1 by @Oberacda in [#165](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/165)
- Merge pull request #164 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.3.0 by @Oberacda in [#164](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/164)
- Merge branch 'main' into update-web-gui-to-release/3.3.0 by @Oberacda
- Merge pull request #163 from fzi-forschungszentrum-informatik/fix_memory_subscriber by @Oberacda in [#163](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/163)
- Merge pull request #161 from Doomse/node-meta-fix by @Oberacda in [#161](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/161)
- Merge pull request #162 from Doomse/enum-values by @Oberacda in [#162](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/162)
- Merge pull request #159 from Doomse/type-hints by @Oberacda in [#159](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/159)

### Removed
- Remove non-functional enum node by @Doomse


## [0.2.0] - 2025-01-23

### Added
- Add handler to allow the loading of old trees. by @Oberacda
- Add default values to Ros Message types by @Doomse
- Add action name discovery by @Doomse
- Add Ros Service types by @Doomse
- Added missing ) by @nspielbau
- Added showcase gif by @nspielbau
- Added line between icons and header by @nspielbau

### Changed
- Restructure MessageType publlishing by @Doomse
- Keep maths types in helpers.py for compatibility by @Doomse
- Implement Ros Topic types by @Doomse
- Implement Ros Action types and baseclasses by @Doomse
- Publish names and types of existing topics and services. Actions still missing by @Doomse
- Move math types to new location by @Doomse
- Implement FilePath type by @Doomse
- Updated Changelog by @nspielbau
- Updated gif by @nspielbau
- Update CI workflow. by @Oberacda
- Update Web-GUI to release/3.2.3 by @Oberacda
- Update Web-GUI to release/3.1.0 by @Oberacda in [#150](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/150)

### Fixed
- Fix invalid default value. by @Oberacda
- Fix test_wait_for_service_input by @Oberacda
- Fix test_wait_for_service by @Oberacda
- Fix test_service_input by @Oberacda
- Fix test_service by @Oberacda
- Fix test_message_to_fields by @Oberacda
- Fix test_message_from_dict_const by @Oberacda
- Fix test_message_from_dict by @Oberacda
- Fix test_fields_to_message by @Oberacda
- Fixed up README by @nspielbau

### Merged
- Merge pull request #154 from fzi-forschungszentrum-informatik/type-system-web-update by @Oberacda in [#154](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/154)
- Merge pull request #156 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.2.2 by @Oberacda
- Merge branch 'type-system-web-update' into update-web-gui-to-release/3.2.2 by @Oberacda
- Merge pull request #151 from Doomse/type_system by @Oberacda
- Merge pull request #143 from fzi-forschungszentrum-informatik/rework_landing_page by @Oberacda in [#143](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/143)
- Merge pull request #158 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.2.3 by @Oberacda in [#158](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/158)

### Removed
- Remove empty action. by @Oberacda
- Remove checks that are now redundant by @Doomse


## [0.1.1] - 2024-11-04

### Added
- Added documentation fix to changelog by @nspielbau
- Added mock imports for autodoc by @nspielbau
- Added tree storage path documentation by @nspielbau
- Add codecov config. by @Oberacda in [#135](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/135)
- Added manual documentation build to doc dir by @nspielbau in [#134](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/134)

### Changed
- Update Web-GUI to release/3.0.0 by @Oberacda
- Bumped doc version by @nspielbau in [#133](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/133)

### Fixed
- Fixed module path for autodoc by @nspielbau
- Fix documentation makefile back to tabs. by @Oberacda

### Merged
- Merge pull request #146 from fzi-forschungszentrum-informatik/update-web-gui-to-release/3.0.0 by @Oberacda in [#146](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/146)
- Merge pull request #139 from fzi-forschungszentrum-informatik/fix_documentation_build by @Oberacda in [#139](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/139)
- Merge pull request #141 from fzi-forschungszentrum-informatik/doc_tree_storage_path by @Oberacda in [#141](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/141)
- Merge pull request #126 from sea-bass/distros-in-ci by @Oberacda in [#126](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/126)
- Merge branch 'main' into distros-in-ci by @Oberacda

### Removed
- Removed faulty header by @nspielbau
- Removed shoving ref which was not defined by @nspielbau
- Removed some !-marks by @nspielbau
- Remove docu build from README by @nspielbau


## [0.1.0] - 2024-10-28

### Added
- Added more complex tutorials by @nspielbau
- Added ROS interfacing by @nspielbau in [#118](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/118)
- Adding subtree section by @nspielbau
- Added custom launch documentation by @nspielbau in [#102](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/102)
- Added clarification on GetAttr usage by @nspielbau in [#92](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/92)
- Added time module to changelog by @nspielbau
- Added time module and TimeNow Node by @nspielbau in [#111](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/111)
- Add set_state function to ensure valid state tran. by @Oberacda in [#107](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/107)
- Added subtree_manager to overwritten inits by @nspielbau in [#99](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/99)
- Added new module list function by @nspielbau
- Add exception for jazzy on workflow after install script. by @Oberacda
- Add jazzy to ici config. by @Oberacda
- Added tree_storage_path as launch argument by @t-schnell in [#86](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/86)
- Add tests. by @Oberacda in [#82](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/82)
- Added comments by @mdeitersen in [#50](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/50)
- Added missing copyright text for test classes by @mdeitersen
- Added subtree manager to all nodes by @mdeitersen
- Added new SubtreeInfo Topic by @mdeitersen
- Added extension tests by @nspielbau in [#74](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/74)
- Added to examples by @nspielbau
- Added internal tests by @nspielbau
- Added Init tests for NodeConfig by @nspielbau
- Added tests for OptionRef class by @nspielbau
- Add Jazzy and Rolling to CI by @sea-bass
- Add Jazzy / Ubuntu 24.04 to Git issue template by @sea-bass

### Changed
- Made RandomIntInput consistent with RandomInt by @nspielbau in [#123](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/123)
- Updated RandomInt Node description by @nspielbau
- Adjusted RandomInt to make more sense by @nspielbau
- Reworked getting started and added tutorials by @nspielbau
- Reworked header levels by @nspielbau
- Renamed chapter to fit better by @nspielbau
- Updated landing page by @nspielbau
- Add missing attributes in topic.py by @Oberacda
- Switch to MultiThreadedExecutor. by @Oberacda
- Cleanly shut down tree node. by @Oberacda
- Updated CHANGELOG by @nspielbau in [#103](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/103)
- Adapted tests for Service nodes by @mdeitersen in [#100](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/100)
- Show action goal, result and feedback fields in Action nodes by @mdeitersen
- Directly show service request and response fields in Service nodes. by @Oberacda
- Enabled default value on launch by @nspielbau in [#93](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/93)
- Overwrite output method by @nspielbau in [#89](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/89)
- Result can now be processed when using ABC Actions by @nspielbau in [#84](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/84)
- Ignore parameters module in test_import.py by @Oberacda
- Reset wait nodes before shutdown. by @Oberacda
- Deleted duplicated conditional statement by @mdeitersen
- Replaced NodeDiagnostics message with DiagnosticStatus message by @mdeitersen
- Replaced  SetExecutionMode.srv for setting collect_node_diagnostics in DebugManager and publish_subtrees in SubtreeManager  with two seperate services by @mdeitersen
- Moved subtree manager from debug manager to another class by @mdeitersen
- Okay this seems to work, idk why by @nspielbau
- Changed order inside asserts by @nspielbau
- Update Web-GUI to release/2.0.3 by @Oberacda in [#61](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/61)
- Updated TOC in all CONTRIBUTING.md files
- Update Web-GUI to release/2.0.4 by @Oberacda
- Update Web-GUI to release/2.0.5 by @Oberacda
- Update industrial_ci.yml by @sea-bass
- Update industrial_ci.yml by @sea-bass
- Update BUG-REPORT.yml by @sea-bass in [#117](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/117)
- Packages list only ever contained one item by @Oberacda

### Fixed
- Fixed package name by @nspielbau in [#125](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/125)
- Fix bug: unable remove Action node with initial default values by @mdeitersen in [#120](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/120)
- Fix invalid usage of self.tree.state by @Oberacda
- Fix undeclared state variable in action node. by @Oberacda
- Fix missing check for feedback in action node. by @Oberacda
- Fix self.name is None error. by @Oberacda
- Fix service call mock assert. by @Oberacda
- Fix style issues. by @Oberacda
- Fix error with subtree initializations. by @Oberacda
- Fix symlink install parameter_library bug. by @Oberacda
- Fixed optional options to be a list by @nspielbau
- Fixed typo by @nspielbau
- Fixed assert by @nspielbau
- Fix cancellation of goal. by @Oberacda in [#63](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/63)
- Fix variable names in action node. by @Oberacda
- Fix action state machine. by @Oberacda
- Fix action broken status after succeeding. by @Oberacda
- Fixed imports in docs: creating_node_classes.rst in [#62](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/62)
- Fixed Python version by @nspielbau

### Merged
- Merge pull request #131 from fzi-forschungszentrum-informatik/dev by @Oberacda in [#131](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/131)
- Merge pull request #95 from fzi-forschungszentrum-informatik/fb_fix_module_list by @Oberacda in [#95](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/95)
- Merge pull request #78 from fzi-forschungszentrum-informatik/fb_remove_debug_code by @Oberacda in [#78](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/78)
- Merge branch 'dev' into fb_remove_debug_code by @Oberacda
- Merge branch 'fb_remove_debug_code' into update-web-gui-to-release/2.0.5 by @Oberacda in [#77](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/77)
- Merge branch 'fix_create_subtree_from_nodes' into update-web-gui-to-release/2.0.4 by @Oberacda in [#76](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/76)

### Removed
- Remove _setting_up variable. by @Oberacda
- Remove unneeded passthrough parameter. by @Oberacda
- Remove Client import in action.py by @Oberacda
- Remove verbose logging. by @Oberacda
- Remove top-level exception catching. by @Oberacda
- Remove unnecessary logging. by @Oberacda in [#97](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/97)
- Remove unnecessary logging messages. by @Oberacda
- Remove ament_flake8 as it conflicts with black. by @Oberacda
- Remove print statements. by @Oberacda in [#80](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/80)
- Removed NodeDiagnostics.msg from CmakeLists by @mdeitersen
- Removed logger information which was temporarily added for debugging purposes by @mdeitersen
- Removed debugging functionalities from DebugManager and DebugInfo by @mdeitersen
- Removed modify_breakpoints by @mdeitersen
- Remove ament_auto_lint from ros_bt_py_web_gui by @Oberacda

## New Contributors
* @sea-bass made their first contribution

## [0.0.1] - 2024-02-22

### Added
- Added basic template for CHANGELOG by @nspielbau in [#54](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/54)
- Add documentation link to README. by @Oberacda
- Add missing theme dependency/ by @Oberacda
- Add missing dependency by @Oberacda
- Add sphinx doc build job. by @Oberacda
- Add Github Actions CI by @Oberacda in [#48](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/48)
- Added tests for debug_manager to test subtree state by @mdeitersen
- Added test for ros_helpers.py by @mdeitersen
- Added test cases for helpers.py by @mdeitersen
- Add codecov github action by @Oberacda
- Add coverage reporting. by @Oberacda
- Add github workflow and issue templates by @Oberacda
- Add msg type to GetMessageFields message. by @Oberacda
- Add rules to make rolling optional. by @Oberacda
- Add non-ros nodes to coverage config. by @Oberacda
- Add tests for param nodes by @Oberacda
- Add tests for wait_for_service nodes by @Oberacda
- Add tests for Service node by @Oberacda
- Add tests for ServiceInput by @Oberacda
- Add ros_node reference to Node instances by @Oberacda
- Add note about generate_param_lib and symlink_install by @fmauch
- Add getting started section by @Oberacda
- Add sphinx based documentation. by @Oberacda
- Add README.md by @Oberacda
- Add url printer for web gui by @Oberacda
- Add web_gui to ros_bt_py launch file. by @Oberacda
- Add web_gui and launch files by @Oberacda
- Add upstream workspace for generate_parameter_library by @Oberacda
- Add upstream workspace for generate_parameter_library by @Oberacda
- Add .coverage to gitignore by @Oberacda
- Add tests for package_manager.py by @Oberacda
- Add internal Gitlab CI to packages by @fmauch
- Add ament_lint linters to pre-commit. by @Oberacda

### Changed
- Update version numbers to match changelog. by @Oberacda
- Update Web-GUI to release/2.0.2 in [#57](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/57)
- Perform unwiring first before removing node by @mdeitersen in [#53](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/53)
- Set all node states to shutdown and tree state to idle when saving a tree by @mdeitersen in [#51](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/51)
- Only upload single test report and tag code coverage by @Oberacda
- Update Readme. by @Oberacda
- Update industrial_ci.yml by @Oberacda
- Update industrial_ci.yml by @Oberacda
- Update industrial_ci.yml by @Oberacda
- Update industrial_ci.yml by @Oberacda
- Update industrial_ci.yml by @Oberacda
- Prepare GitHub Release. by @Oberacda
- Implement proper type completion for the JSON Editor. by @Oberacda
- Update webgui JSON input. by @Oberacda
- Implement save_to_path fuction by @Oberacda
- Use double backticks for referencing code snipptets in RST by @fmauch
- Port file.py nodes by @Oberacda
- Port throttle.py nodes. by @Oberacda
- Port subtree.py by @Oberacda
- Port action.py node by @Oberacda
- Remote test case that failes due to changes in rolling by @Oberacda
- Port topic.py by @Oberacda
- Port enum.py and add tests by @Oberacda
- Update pytest config. by @Oberacda
- Port message_from_dict nodes by @Oberacda
- Port message_converters.py by @Oberacda
- Port RosParamInput and RosParamOptionDefaultInput by @Oberacda
- Port RosParamOption by @Oberacda
- Change code coverage report file paths by @Oberacda
- Use relative paths in coberatura CI report. by @Oberacda
- Enable reports for test coverage by @Oberacda
- Update ci config by @Oberacda
- Port service nodes by @Oberacda
- Import node modules in __init__.py by @Oberacda
- Adapted to new folder structure for src files by @t-schnell
- Porting of ROS independent nodes
- Print web gui URL during startup. by @Oberacda
- Switch to master ci_scripts branch. by @Oberacda
- Switch to fix_coverage_eos2 CI branch. by @Oberacda
- Update web_gui to use new interfaces. by @Oberacda
- Configure pytest to respect generated code. by @Oberacda
- Switch to generate_parameter_library by @Oberacda
- Make tree node executable. by @Oberacda
- Port tree_node.py by @Oberacda
- Port tree_manager.py by @Oberacda
- Port package_manager.py by @fmauch
- Port package_manager.py by @Oberacda
- Try importing each submodule from ros_bt_py by @Oberacda
- Try importing each submodule from ros_bt_py by @fmauch
- Port node class and related code by @fmauch
- Port debug_manager.py by @Oberacda
- Port node.py by @Oberacda
- Port node_config.py by @Oberacda
- Port node_data.py by @Oberacda
- Port helper functions by @fmauch
- Run linters as part of pre-commit not colcon test. by @Oberacda
- Ignore invalid warning from flake8. by @Oberacda
- Port ros_helpers class. by @Oberacda
- Port helpers.py by @Oberacda
- Port exceptions.py by @Oberacda
- Initial commit. by @Oberacda

### Fixed
- Fix invalid subtree creation while saving a tree. by @Oberacda in [#58](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/58)
- Fix missing parameters in nodes. by @Oberacda
- Fix github variables error by @Oberacda
- Fix github variables error by @Oberacda
- Fix github variable problem. by @Oberacda
- Fix missing package.xml dependencies. by @Oberacda
- Fix ci configuration. by @Oberacda
- Fix inconsistent style. by @Oberacda
- Fix missing parameter in PackageManager tests by @Oberacda
- Fixed code snippets by @nspielbau
- Fix subtree constructor by @Oberacda
- Fix service destruction error. by @Oberacda
- Fix topic destruction_error by @Oberacda
- Fix message converters drag and drop. by @Oberacda
- Fix service call and message autocompletion by @Oberacda
- Fix error with invalid ros_node reference by @Oberacda
- Fix node imports and tree_manager service calls. by @Oberacda
- Fix gitlab-ci.yml by @Oberacda
- Fix author in latex report by @Oberacda
- Fix CI errors for web_gui. by @Oberacda
- Fix pipeline test reporting. by @Oberacda
- Fix pipeline test reporting. by @Oberacda
- Fix service callback signatures in package_manager. by @Oberacda
- Fix parameter loading and add launchfile. by @Oberacda
- Fix type errors. by @Oberacda
- Fix pyproject.toml by @Oberacda
- Fix toml error in CI. by @Oberacda
- Fix node import by @fmauch
- Fix code style issues. by @Oberacda
- Fix code style issues. by @Oberacda
- Fix missing import in interfaces CMakeLists.txt by @Oberacda
- Fix CI configuration and add pre-commit to CI. by @Oberacda
- Fix pipeline config. by @Oberacda

### Merged
- Merge pull request #59 from fzi-forschungszentrum-informatik/dev by @Oberacda in [#59](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/59)
- Merge branch 'main' into dev by @Oberacda
- Merge pull request #47 from mdeitersen/add_helper_tests by @Oberacda in [#47](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/47)
- Merge branch 'main' into add_helper_tests by @Oberacda
- Merge branch 'prepare_github_release' into 'main' by @Oberacda
- Merge branch 'main' into fb_web_gui_json_editor_type_completion by @Oberacda
- Merge branch 'main' into fix_subtree_constructor by @Oberacda
- Merge branch 'fix_ci_rolling_allow_to_fail' into 'main'
- Merge branch 'fb_load_save' into 'main' by @fmauch
- Merge branch 'fix_installation_code_snippets' into 'main'
- Merge branch 'fix_destruction_error' into 'main' by @fmauch
- Merge branch 'port_ros_nodes' into 'main' by @fmauch
- Merge branch 'main' into 'port_ros_nodes' by @Oberacda
- Merge branch 'update_readme' into 'main' by @Oberacda
- Merge branch 'fb_add_basic_nodes' into 'main' by @Oberacda
- Merge branch 'add_documentation' into 'main' by @Oberacda
- Merge branch 'add_web_gui' into 'main' by @Oberacda
- Merge branch 'port_tree_manager' into 'main' by @Oberacda

### Removed
- Remove dependencies that are not covered by rosdep by @Oberacda in [#49](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/49)
- Removed fix_yaml by @mdeitersen
- Remove usage of an invalid constructor for the subtree class. by @Oberacda
- Remove old warning from README by @fmauch
- Remove unused imports by @Oberacda
- Remove test parameter that is ros2 version dependent. by @Oberacda

## New Contributors
* @Oberacda made their first contribution in [#59](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/59)
* @nspielbau made their first contribution in [#54](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/54)
* @ made their first contribution in [#57](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/pull/57)
* @fmauch made their first contribution
* @t-schnell made their first contribution

[unreleased]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/compare/v0.4.0..HEAD
[0.4.0]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/compare/v0.3.0..v0.4.0
[0.3.0]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/compare/v0.2.0..v0.3.0
[0.2.0]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/compare/v0.1.1..v0.2.0
[0.1.1]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/compare/v0.1.0..v0.1.1
[0.1.0]: https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/compare/v0.0.1..v0.1.0

<!-- generated by git-cliff -->
