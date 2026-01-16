# main_con.launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --------------------
    # Paths
    # --------------------
    nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    # --------------------
    # Launch Arguments
    # --------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('/home/zero/maze.yaml')
    )

    # --------------------
    # Navigation2 bringup
    # --------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml
        }.items(),
    )

    # --------------------
    # Main Controller
    # --------------------
    main_controller_node = Node(
        package='my_second_pkg',
        executable='main_controller',
        name='main_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,

        nav2_launch,
        main_controller_node
    ])
