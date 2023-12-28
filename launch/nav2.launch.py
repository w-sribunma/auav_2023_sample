from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from launch_ros.actions import PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
]

def generate_launch_description():
    pkg_auav_nav2 = get_package_share_directory('auav_2023_sample')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_x500_description = get_package_share_directory('x500_description')

    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_auav_nav2, 'config', 'nav2.yaml']),
        description='Nav2 parameters')

    namespace_arg = DeclareLaunchArgument(
                        'namespace',
                        default_value='',
                        description='Robot namespace')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2 = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_robot_state_pub': 'True',
                              'use_composition': 'False',
                              'log_level': LaunchConfiguration('log_level'),
                              'params_file': LaunchConfiguration('params_file')}.items()),
    ])

    tf_map_odom =  Node(
           package='tf2_ros',
           output='screen',
           executable='static_transform_publisher',
           arguments=["--frame-id", "map", "--child-frame-id", "odom"]
    )

    # x500_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': Command(['xacro', ' ', xacro_path])
    #     }]
    # )

    x500_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_x500_description, 'launch', 'robot_description.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}
    )


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_params_arg)
    ld.add_action(namespace_arg)
    ld.add_action(nav2)
    ld.add_action(tf_map_odom) #RECHECK IF THIS IS neccessary
    # ld.add_action(x500_state_publisher)
    return ld
