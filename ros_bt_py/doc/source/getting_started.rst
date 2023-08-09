Getting Started
===============

Installation
------------

Installing from binary packages
+++++++++++++++++++++++++++++++

Installation form binary packages is currently not supported, but will be added in the future.

Installing from source
++++++++++++++++++++++

To install `ros_bt_py` from source you need to follow these steps:

1. Create a colcon workspace. (If you already have a colcon workspace you can skip these steps.)

  1. Create a new folder with the name of the workspace, e.g. `test_ws`
  2. Inside this folder a folder called `src`.
  3. Source your ROS2 installation `source /opt/ros/*ros2 version*/setup.[bash|zsh|sh]`
  4. Run `colcon build`

2. Clone the `ros_bt_py` repo_ from the `src` folder.
3. Install all dependencies via `rosdep install --from-paths . --ignore-src -y`.
4. From the `test_ws` folder call `colcon build`.
5. Run `source install/setup.[bash|zsh|sh]` to load the build packages.


Now `ros_bt_py` is build and installed in your workspace and can be used.

.. _repo: https://ids-git.fzi.de/ros2/ros_bt_py

Running ros_bt_py
-----------------

The command

.. code-block:: bash

  ros2 launch ros_bt_py ros_bt_py.launch.py enable_web_interface:=True


will start a BT server and the rosbridge and webserver needed for the GUI.
Afterwards, you can open `http://localhost:8085/index.html` to use the editor.

Launch Options
++++++++++++++

.. list-table:: Launch Options
   :widths: auto
   :header-rows: 1

   * - Launch Argument
     - Description
     - Default Value
     - Launch File
   * - robot_namespace
     - Namespace to launch all ros_bt_py nodes in!
     - /
     - ros_bt_py
   * - node_modules
     - List of python packages that contain nodes to be loaded on startup.
     - "['ros_bt_py.nodes','ros_bt_py.ros_nodes']"
     - ros_bt_py
   * - enable_web_interface
     - Start web GUI on startup.
     - False
     - ros_bt_py
   * - show_traceback_on_exception
     - Show error traceback when an exception rises.
     - True
     - ros_bt_py
   * - diagnostics_frequency_hz
     - Publishing frequency for diagnostics msgs.
     - 1.0
     - ros_bt_py
   * - load_default_tree
     - Load the default tree on startup!
     - False
     - ros_bt_py
   * - load_default_tree_permissive
     - Load the default tree in permissive mode on startup!
     - False
     - ros_bt_py
   * - default_tree_path
     - Path to the default tree to load on startup!
     - ""
     - ros_bt_py
   * - default_tree_tick_frequency_hz
     - Frequency with which to tick the default tree loaded on startup!
     - 10.0
     - ros_bt_py
   * - default_tree_control_command
     - Command to execute per default after loading the default tree on startup!
     - 2
     - ros_bt_py
   * - web_server_port
     - Port to use for the web interface.
     - 8085
     - ros_bt_py & ros_bt_py_interfaces
   * - web_server_address
     - IP address to use for the web interface. Default value uses all IP addresses of the host.
     - 0.0.0.0
     - ros_bt_py & ros_bt_py_interfaces

Stand-alone Web Interface
+++++++++++++++++++++++++

The web interface can be launched stand alone of the library, using the following command:

.. code-block:: bash

  ros2 launch ros_bt_py_web_gui ros_bt_py_web_gui.launch.py web_server_port:=8085 web_server_address:=0.0.0.0
