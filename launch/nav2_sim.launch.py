import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ----- Paths -----
    world_file = '/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world'
    xacro_file = '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf'
    sdf_file = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'
    map_file = '/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml'
    nav2_params_file = '/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml'
    rviz_config_file = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'

    nav2_bringup_launch = '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py'

    # ----- Robot description from xacro -----
    robot_description = xacro.process_file(xacro_file).toxml()

    # ----- Gazebo server -----
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # ----- Robot state publisher -----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ----- Spawn robot into Gazebo -----
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', sdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-timeout', '120.0',
        ],
        output='screen',
    )

    # ----- Nav2 bringup (includes map_server, amcl, planner, controller, BT, etc.) -----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
    )

    # ----- RViz -----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
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
        TimerAction(
            period=20.0,
            actions=[nav2, rviz],
        ),
    ])
