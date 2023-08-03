import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    web_server_port_value = LaunchConfiguration("web_server_port")
    web_server_port_launch_arg = DeclareLaunchArgument(
        "web_server_port",
        default_value="8085",
        description="Port to use for the web server, serving the ros_bt_py web gui.",
    )

    web_server_address_value = LaunchConfiguration("web_server_address")
    web_server_address_launch_arg = DeclareLaunchArgument(
        "web_server_address",
        default_value="0.0.0.0",
        description="Network address to use for the web server, serving the ros_bt_py web gui.",
    )
    web_folder = os.path.join(get_package_share_directory("ros_bt_py_web_gui"), "html")

    web_server = ExecuteProcess(
        cmd=[
            [
                "python3 ",
                "-m http.server ",
                "-b ",
                web_server_address_value,
                " -d ",
                web_folder,
                " ",
                web_server_port_value,
            ]
        ],
        shell=True,
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

    print_editor_url_node = Node(
        package="ros_bt_py_web_gui",
        namespace="ros_bt_py_web_gui",
        executable="show_editor_url.py",
        name="show_editor_url",
        parameters=[
            {"hostname": web_server_address_value},
            {"port": web_server_port_value},
        ],
    )

    return LaunchDescription(
        [
            web_server_address_launch_arg,
            web_server_port_launch_arg,
            web_server,
            rosbridge_server,
            print_editor_url_node,
        ]
    )
