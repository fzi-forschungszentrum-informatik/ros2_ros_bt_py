.. _advanced-launch-config:

#############################
Advanced Launch Configuration
#############################

Using the standard ros_bt_py launch file is all you need for designing and executing BTs with the
standard bt_py nodes.
Utilizing the Package Loader in the GUI you can also extend this to custom Node Classes to enhance
your bt_py experience!

While this is nice for development, using BTs in production has other constraints and conditions,
such as automatically loading custom node classes or starting a tree on launch.

In general you will want to create a custom package for your launch files, similar to the custom
nodes, to keep your ``ros_bt_py`` repository clean.

**************************************
Using Custom Node Classes in a Project
**************************************

After writing and testing all the node classes needed inside ``your_awesome_package`` you will need
to include them when launching ``ros_bt_py`` to actually use them.

All you need to do is to write a new launch file and overwrite the ``module_list`` argument to
include your node modules::

  from launch import LaunchDescription
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  from launch_ros.substitutions import FindPackageShare

  def generate_module_list():
      module_list = [
          "ros_bt_py.nodes",
          "ros_bt_py.ros_nodes",
          "your_awesome_package.nodes",
      ]
      return str(module_list)

  def generate_launch_description():
      ros_bt_py_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              [
                  FindPackageShare("ros_bt_py"),
                  "/launch",
                  "/ros_bt_py.launch.py",
              ]
          ),
          launch_arguments={"node_modules": generate_module_list()}.items(),
      )

      return LaunchDescription(
          [
              ros_bt_py_launch
          ]
      )

Because ``LaunchDescription`` only takes strings as launch arguments the custom
``generate_module_list`` function is used to allow for adding multiple new modules while also
allowing for line breaks for readability.

********************************
Other Important Launch Arguments
********************************

Other relevant launch arguments you might find useful:

.. list-table:: Launch Options
   :widths: auto
   :header-rows: 1

   * - Launch Argument
     - Description
     - Default Value
     - Tip
   * - enable_web_interface
     - Start web GUI on startup.
     - False
     - For working in production the interface is not necessary
   * - load_default_tree
     - Load the default tree on startup
     - False
     - Needs to be set to true to load a tree
   * - default_tree_path
     - Path to the default tree to load on startup!
     - ""
     - path to load the tree, can be used with ``file://`` or ``package://``
   * - default_tree_control_command
     - Command to execute per default after loading the default tree on startup!
     - 2
     - DO_NOTHING = 0, TICK_ONCE = 1, TICK_PERIODICALLY = 2, TICK_UNTIL_RESULT = 3
   * - tree_storage_paths
     - Specify the storage paths for your trees when using "Save to File"
     - "['$HOME/.ros']"
     - Use this to save trees directly to your tree repository
