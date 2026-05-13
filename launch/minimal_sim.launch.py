from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    world_file = '/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world'

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

    return LaunchDescription([
        gzserver,
        gzclient,
    ])
