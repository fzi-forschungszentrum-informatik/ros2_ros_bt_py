from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    robot_namespace_launch_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="/",
        description="Namespace to use for the ros_bt_py library",
    )
    robot_namespace_value = LaunchConfiguration("robot_namespace")

    node_modules_launch_arg = DeclareLaunchArgument(
        "node_modules",
        default_value="['ros_bt_py.nodes','ros_bt_py.ros_nodes']",
        description="Default python modules from which to load node definitions",
    )
    node_modules_value = LaunchConfiguration("node_modules")

    enable_web_interface_launch_arg = DeclareLaunchArgument(
        "enable_web_interface",
        default_value="False",
        description="Enable the ros_bt_py web GUI",
    )
    enable_web_interface_value = LaunchConfiguration("enable_web_interface")

    show_traceback_on_exception_launch_arg = DeclareLaunchArgument(
        "show_traceback_on_exception",
        default_value="True",
        description="Show error traceback when an exception rises",
    )
    show_traceback_on_exception_value = LaunchConfiguration(
        "show_traceback_on_exception"
    )

    diagnostics_frequency_hz_launch_arg = DeclareLaunchArgument(
        "diagnostics_frequency_hz",
        default_value="10.0",
        description="Publishing frequency for diagnostics msgs.",
    )
    diagnostics_frequency_hz_value = LaunchConfiguration("diagnostics_frequency_hz")

    load_default_tree_launch_arg = DeclareLaunchArgument(
        "load_default_tree",
        default_value="False",
        description="Load the default tree on startup!",
    )
    load_default_tree_value = LaunchConfiguration("load_default_tree")

    load_default_tree_permissive_launch_arg = DeclareLaunchArgument(
        "load_default_tree_permissive",
        default_value="False",
        description="Load the default tree in permissive mode on startup!",
    )
    load_default_tree_permissive_value = LaunchConfiguration(
        "load_default_tree_permissive"
    )

    default_tree_path_launch_arg = DeclareLaunchArgument(
        "default_tree_path",
        default_value="",
        description="Path to the default tree to load on startup!",
    )
    default_tree_path_value = LaunchConfiguration("default_tree_path")

    default_tree_tick_frequency_hz_launch_arg = DeclareLaunchArgument(
        "default_tree_tick_frequency_hz",
        default_value="10.0",
        description="Frequency with which to tick the default tree loaded on startup!",
    )
    default_tree_tick_frequency_hz_value = LaunchConfiguration(
        "default_tree_tick_frequency_hz"
    )

    default_tree_control_command_launch_arg = DeclareLaunchArgument(
        "default_tree_control_command",
        default_value="0",
        description="Command to execute per default after loading the default tree on startup!",
        choices=["0", "1", "2", "3"],
    )
    default_tree_control_command_value = LaunchConfiguration(
        "default_tree_control_command"
    )

    ros_bt_py_node = Node(
        package="ros_bt_py",
        executable="tree_node",
        namespace=robot_namespace_value,
        parameters=[
            {"node_modules": node_modules_value},
            {"show_traceback_on_exception": show_traceback_on_exception_value},
            {"diagnostics_frequency_hz": diagnostics_frequency_hz_value},
            {"default_tree/load_default_tree": load_default_tree_value},
            {
                "default_tree/load_default_tree_permissive": load_default_tree_permissive_value
            },
            {"default_tree/tree_path": default_tree_path_value},
            {"default_tree/tick_frequency_hz": default_tree_tick_frequency_hz_value},
            {"default_tree/control_command": default_tree_control_command_value},
        ],
    )

    ros_bt_py_web_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ros_bt_py_web_gui"),
                "/launch",
                "/ros_bt_py_web_gui.launch.py",
            ]
        ),
        condition=IfCondition(enable_web_interface_value),
    )

    return LaunchDescription(
        [
            robot_namespace_launch_arg,
            node_modules_launch_arg,
            enable_web_interface_launch_arg,
            show_traceback_on_exception_launch_arg,
            diagnostics_frequency_hz_launch_arg,
            load_default_tree_launch_arg,
            load_default_tree_permissive_launch_arg,
            default_tree_path_launch_arg,
            default_tree_tick_frequency_hz_launch_arg,
            default_tree_control_command_launch_arg,
            ros_bt_py_node,
            ros_bt_py_web_gui_launch,
        ]
    )
