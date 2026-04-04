from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("hawkeye")

    ortho_map = LaunchConfiguration("ortho_map")
    config = LaunchConfiguration("config")
    lidar_topic_name = LaunchConfiguration("lidar_topic_name")
    extra_args = LaunchConfiguration("extra_args")
    ws_mode = LaunchConfiguration("ws_mode")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_info = LaunchConfiguration("rviz_info")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ortho_map"),
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution([pkg, "config", "hawkeye.yaml"]),
            ),
            DeclareLaunchArgument("lidar_topic_name", default_value="pointcloud_raw"),
            DeclareLaunchArgument("extra_args", default_value=""),
            DeclareLaunchArgument("ws_mode", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_info",
                default_value=PathJoinSubstitution([pkg, "rviz", "hawkeye.rviz"]),
            ),
            # hawkeye_rt (CopyShiftMode)
            Node(
                package="hawkeye",
                executable="hawkeye_rt",
                name="hawkeye_rt",
                arguments=[ortho_map, config, lidar_topic_name],
                output="screen",
                condition=UnlessCondition(ws_mode),
            ),
            # hawkeye_rt_ws (WeightedShiftMode)
            Node(
                package="hawkeye",
                executable="hawkeye_rt_ws",
                name="hawkeye_rt_ws",
                arguments=[ortho_map, config, lidar_topic_name],
                output="screen",
                condition=IfCondition(ws_mode),
            ),
            # rviz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="hawkeye_rviz",
                arguments=["-d", rviz_info],
                output="log",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
