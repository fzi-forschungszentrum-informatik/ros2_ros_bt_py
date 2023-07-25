import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    web_folder = os.path.join(get_package_share_directory("ros_bt_py_web_gui"), "html")

    web_server = ExecuteProcess(
        cmd=[["python3 ", "-m http.server ", "-d ", web_folder, " 8085"]], shell=True
    )

    rosbridge_server = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                FindPackageShare("rosbridge_server"),
                "/launch",
                "/rosbridge_websocket_launch.xml",
            ]
        )
    )

    return LaunchDescription([web_server, rosbridge_server])
