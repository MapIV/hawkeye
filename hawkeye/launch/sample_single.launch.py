from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("hawkeye")

    sample_dir = LaunchConfiguration("sample_dir")
    lidar_topic_name = LaunchConfiguration("lidar_topic_name")
    extra_args = LaunchConfiguration("extra_args")
    ws_mode = LaunchConfiguration("ws_mode")
    use_rviz = LaunchConfiguration("use_rviz")

    ortho_map = PathJoinSubstitution([sample_dir, "map", "ortho_image", "_orthomap"])
    config = PathJoinSubstitution([sample_dir, "config", "hawkeye.yaml"])
    rviz_info = PathJoinSubstitution([pkg, "rviz", "hawkeye.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("sample_dir", default_value=""),
            DeclareLaunchArgument("lidar_topic_name", default_value="pointcloud_raw"),
            DeclareLaunchArgument("extra_args", default_value=""),
            DeclareLaunchArgument("ws_mode", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, "launch", "hawkeye_rt.launch.py"])
                ),
                launch_arguments={
                    "ortho_map": ortho_map,
                    "config": config,
                    "lidar_topic_name": lidar_topic_name,
                    "extra_args": extra_args,
                    "use_rviz": use_rviz,
                    "rviz_info": rviz_info,
                    "ws_mode": ws_mode,
                }.items(),
            ),
        ]
    )
