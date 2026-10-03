import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription,
    TimerAction, DeclareLaunchArgument, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    controller = LaunchConfiguration('controller').perform(context)
    valid = ('rpp', 'mppi', 'dwb')
    if controller not in valid:
        raise RuntimeError(f"controller must be one of {valid}, got '{controller}'")

    pkg_share = get_package_share_directory('mower_sim')

    world_file = os.path.join(pkg_share, 'worlds', 'empty_field.world')
    xacro_file = '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf'
    sdf_file = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'

    nav2_params_file = os.path.join(pkg_share, 'config', f'nav2_{controller}_fair.yaml')
    map_yaml_file = os.path.join(pkg_share, 'maps', 'empty_map.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mower_view.rviz')
    nav2_bringup_launch = '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'

    print(f"\n[mower_sim] Controller: {controller.upper()}")
    print(f"[mower_sim] Params: {nav2_params_file}\n")

    robot_description = xacro.process_file(xacro_file).toxml()

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen',
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_burger',
                   '-file', sdf_file,
                   '-x', '0.0', '-y', '0.0', '-z', '0.05',
                   '-timeout', '120.0'],
        output='screen',
    )

    ground_truth_tf = Node(
        package='mower_sim',
        executable='ground_truth_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # navigation_launch.py brings up planning/control only - no map_server and no
    # AMCL. Localization comes from ground_truth_tf (map -> odom), but the costmap
    # static layers still need a /map publisher, so run map_server ourselves.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml_file}],
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'node_names': ['map_server']}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.environ.get('GAZEBO_MODEL_PATH', '') +
                  ':/opt/ros/humble/share/turtlebot3_gazebo/models' +
                  ':/opt/ros/humble/share'
        ),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        ground_truth_tf,
        TimerAction(period=20.0, actions=[map_server, map_server_lifecycle,
                                          nav2, rviz]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller',
            default_value='rpp',
            description="Which controller to use: 'rpp', 'mppi', or 'dwb'",
        ),
        OpaqueFunction(function=launch_setup),
    ])
