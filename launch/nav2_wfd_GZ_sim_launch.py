import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [get_package_share_directory('auav_2023_sample'),'config','slam_async.yaml']),
        description='path to editted slam paramaters inside auav_2023_sample package')\

    declare_namespace= DeclareLaunchArgument(
        'namespace',default_value='',description='Robot namespace'
    )

    slam_params = RewrittenYaml(
        source_file=LaunchConfiguration('params'),
        root_key=namespace,
        param_rewrites={},
        convert_types=True
        )
    
    remappings = [
        ('/tf','tf'),
        ('/tf_static','tf_static'),
        ('/scan','/lidar'),
        ('/scan','scan'),
        ('/map', 'map'),
        ('/map_metadata','map_metadata'),
        ('/odom','odom'),
        ('/odom','/px4_1/fmu/out/vehicle_odometry')
    ]

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=remappings
        )
    
    nav2_wfd_node = Node(
        package='nav2_wfd',
        name='nav2_wavefront_frontier_exploration',
        executable='explore',
        remappings=remappings
    )

    tf_odom_base_link =  Node(
        package='tf2_ros',
        output='screen',
        executable='static_transform_publisher',
        arguments=["--frame-id", "odom", "--child-frame-id", "base_footprint"]
    )

    tf_base_footprint_base_link =  Node(
        package='tf2_ros',
        output='screen',
        executable='static_transform_publisher',
        arguments=["--frame-id", "base_footprint", "--child-frame-id", "base_link"]
    )

    tf_gz_base_link =  Node(
        package='tf2_ros',
        output='screen',
        executable='static_transform_publisher',
        arguments=["--frame-id", "base_link", "--child-frame-id", "x500_lidar_1/base_link/lidar_sensor"]
    )


# Transforms for Nav2 Requires the following

# map --> odom provided from slam_toolbox
    # tf_map_odom =  Node(
    #     package='tf2_ros',
    #     output='screen',
    #     executable='static_transform_publisher',
    #     arguments=["--frame-id", "map", "--child-frame-id", "odom"]
    # )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_namespace)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(nav2_wfd_node)
    # ld.add_action(laser)
    ld.add_action(tf_odom_base_link)
    ld.add_action(tf_base_footprint_base_link)
    ld.add_action(tf_gz_base_link)
    # ld.add_action(tf_map_odom)

    return ld
