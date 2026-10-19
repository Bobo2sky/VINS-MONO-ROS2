from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/euroc/euroc_config_800mp.yaml'
    ])


    vins_path = PathJoinSubstitution([
        config_pkg_path,
        'config/../'
    ])

    # Define the node
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )

    # rviz 已移出 launch(8MP 场景渲染耗 CPU);需要可视化时手动启动:
    # ros2 run rviz2 rviz2 -d <config_pkg_share>/config/vins_euroc_rviz.rviz

    return LaunchDescription([
        LogInfo(msg=['[feature tracker launch] config path: ', config_path]),
        feature_tracker_node
    ])