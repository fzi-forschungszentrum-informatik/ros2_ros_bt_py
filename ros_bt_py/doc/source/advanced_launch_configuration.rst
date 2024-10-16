.. _advanced-launch-config:

#############################
Advanced Launch Configuration
#############################

**************************************
Using Custom Node Classes in a Project
**************************************

When adding more complex node classes it is strongly advised, that you create your own Python
package for them, so you don't have to fork (or even have uncommited changes!!) in your
``ros_bt_py`` repository.

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
