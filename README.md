[![ROS Industrial CI](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/actions/workflows/industrial_ci.yml/badge.svg)](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/actions/workflows/industrial_ci.yml)
[![codecov](https://codecov.io/gh/fzi-forschungszentrum-informatik/ros2_ros_bt_py/graph/badge.svg?token=CRF3GMWWG3)](https://codecov.io/gh/fzi-forschungszentrum-informatik/ros2_ros_bt_py)
[![Documentation](https://img.shields.io/badge/Documentation-Github_Pages-blue)](https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/index.html)
---

# Welcome to ros_bt_py!

This is a [Behavior Tree](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)) library meant to be an alternative to [BehaviorTree.cpp](https://www.behaviortree.dev/), [SMACH](http://wiki.ros.org/smach), [FlexBE](http://wiki.ros.org/flexbe) and the like.

It includes a ReactJS-based web GUI and all the building blocks you need to build advanced mission
control Behavior Trees without writing a single line of code!

**ros_bt_py** is a plug-and-play solution for all your ROS 2 deliberation needs including:

* Intuitive interfacing with **ROS Actions** and **Services** through run-time configurable Node Classes
* Explicit and type-safe **Dataflow** utilizing a drag-and-drop based datagraph.
* Easy customization and expansion of the available Node Classes as **Python Modules**

ros_bt_py has been the subject of multiple publications:
* [Distributed Behavior Trees for Heterogeneous Robot Teams](https://arxiv.org/pdf/2309.08253)
* [Behavior Tree Capabilities for Dynamic Multi-Robot Task Allocation with Heterogeneous Robot Teams](https://arxiv.org/pdf/2402.02833)

<p align="center"><img src="bt_py.gif"></p>

## Documentation, Improvements and Community

Learn about how to get started, API and tutorials in [our documentation](https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/index.html).

If you have specific questions or ideas feel free to [write an issue](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/issues/new/choose) or [start a discussion](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/discussions).

## Installation

To actually start using ros_bt_py, you need to install its dependencies first:

```bash
$ cd colcon_workspace
$ rosdep install --from-paths src --ignore-src -r -y
```

Then you can just build the package with your preferred method i.e. `colcon build`

**ros_bt_py is under active development so things on the main branch might not be stable.**

**To make sure you do use a stable version check out the latest [release](https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py/releases).**

## Running

The command
```bash
$ ros2 launch ros_bt_py ros_bt_py.launch.py enable_web_interface:=True
```

will start a BT server and the rosbridge and webserver needed for the
GUI.
Afterwards, you can open `http://localhost:8085/index.html` to use the editor.

### Launch Options

| **Launch Argument**            | **Description**                                                                             | **Default Value**                             |
|--------------------------------|---------------------------------------------------------------------------------------------|-----------------------------------------------|
| robot_namespace                | Namespace to launch all ros_bt_py nodes in!                                                 | /                                             |
| node_modules                   | List of python packages that contain nodes to be loaded on startup.                         | _"['ros_bt_py.nodes','ros_bt_py.ros_nodes']"_ |
| enable_web_interface           | Start web GUI on startup.                                                                   | _False_                                       |
| show_traceback_on_exception    | Show error traceback when an exception rises.                                               | _True_                                        |
| diagnostics_frequency_hz       | Publishing frequency for diagnostics msgs.                                                  | _1.0_                                         |
| load_default_tree              | Load the default tree on startup!                                                           | _False_                                       |
| load_default_tree_permissive   | Load the default tree in permissive mode on startup!                                        | _False_                                       |
| default_tree_path              | Path to the default tree to load on startup!                                                |                                               |
| default_tree_tick_frequency_hz | Frequency with which to tick the default tree loaded on startup!                            | _10.0_                                        |
| default_tree_control_command   | Command to execute per default after loading the default tree on startup!                   | _2_                                           |
| web_server_port                | Port to use for the web interface.                                                          | _8085_                                        |
| web_server_address             | IP address to use for the web interface. _Default value uses all IP addresses of the host._ | _0.0.0.0_                                     |

### Stand-alone Web Interface

The web interface can be launched stand alone of the library, using the following command:

```bash
$ ros2 launch ros_bt_py_web_gui ros_bt_py_web_gui.launch.py web_server_port:=8085 web_server_address:=0.0.0.0
```
