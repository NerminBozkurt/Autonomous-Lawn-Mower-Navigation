import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    world_file = '/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world'
    
    xacro_file = '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf'

    robot_description = xacro.process_file(xacro_file).toxml()

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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
        ],
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
    ])