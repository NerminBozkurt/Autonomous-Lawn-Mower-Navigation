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
    odometry = LaunchConfiguration('odometry').perform(context)
    if odometry not in ('encoder', 'ground_truth'):
        raise RuntimeError(
            f"odometry must be 'encoder' or 'ground_truth', got '{odometry}'")

    pkg_share = get_package_share_directory('mower_sim')

    world_file = os.path.join(pkg_share, 'worlds', 'empty_field.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mower.urdf.xacro')

    robot_name = 'mower'

    nav2_params_file = os.path.join(pkg_share, 'config', f'nav2_{controller}_fair.yaml')
    map_yaml_file = os.path.join(pkg_share, 'maps', 'empty_map.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mower_view.rviz')
    nav2_bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    print(f"\n[mower_sim] Controller: {controller.upper()}")
    print(f"[mower_sim] Odometry: {odometry}")
    print(f"[mower_sim] Params: {nav2_params_file}\n")

    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    # encoder: wheel_odometry integrates the wheel joint angles and nothing
    # corrects the drift (map -> odom stays the identity), i.e. dead
    # reckoning. ground_truth: the diff drive plugin publishes odometry from
    # Gazebo's true pose and map -> odom is exact, i.e. perfect localization.
    use_encoders = odometry == 'encoder'
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_lidar': use_lidar,
                  'ground_truth_odometry': 'false' if use_encoders else 'true'}
    ).toxml()

    # gazebo_params.yaml raises the /clock rate from its 10 Hz default.
    gazebo_params_file = os.path.join(pkg_share, 'config', 'gazebo_params.yaml')
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file,
             '--ros-args', '--params-file', gazebo_params_file, '--'],
        output='screen',
    )
    # headless:=true drops both GUIs (scripted benchmark runs); gazebo_gui and
    # rviz switch them off one at a time.
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    use_gzclient = not headless and \
        LaunchConfiguration('gazebo_gui').perform(context) == 'true'
    use_rviz = not headless and \
        LaunchConfiguration('rviz').perform(context) == 'true'
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawn from /robot_description rather than a model file: the mower is
    # described by our own xacro, which carries its Gazebo plugins with it.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name,
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05',
                   '-timeout', '120.0'],
        output='screen',
    )

    # Gazebo reports the model pose in the root link's frame, which is
    # base_footprint, so the correction must be composed against that frame.
    ground_truth_tf = Node(
        package='mower_sim',
        executable='ground_truth_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_name': robot_name,
                     'base_frame': 'base_footprint',
                     'correct_odom': not use_encoders}],
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

    wheel_odometry = Node(
        package='mower_sim',
        executable='wheel_odometry',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Ground-truth track of the robot, drawn in RViz next to /coverage_path.
    robot_trail = Node(
        package='mower_sim',
        executable='robot_trail_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return [
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.environ.get('GAZEBO_MODEL_PATH', '') + ':' +
                  os.path.dirname(get_package_share_directory('gazebo_ros'))
        ),
        gzserver,
        *([gzclient] if use_gzclient else []),
        robot_state_publisher,
        spawn_entity,
        ground_truth_tf,
        *([wheel_odometry] if use_encoders else []),
        robot_trail,
        TimerAction(period=20.0, actions=[map_server, map_server_lifecycle, nav2,
                                          *([rviz] if use_rviz else [])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller',
            default_value='rpp',
            description="Which controller to use: 'rpp', 'mppi', or 'dwb'",
        ),
        DeclareLaunchArgument(
            'odometry',
            default_value='encoder',
            description=("Odometry source: 'encoder' (wheel encoders, dead "
                         "reckoning) or 'ground_truth' (perfect localization)"),
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Simulate the 2D lidar (false drops the ray sensor)',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run without gzclient and RViz',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Start the Gazebo client window',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with rviz/mower_view.rviz',
        ),
        OpaqueFunction(function=launch_setup),
    ])
