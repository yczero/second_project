import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value='/home/zero/maze.yaml'
    )

    nav2_params_path = os.path.join(
        get_package_share_directory('my_second_pkg'),
        'config',
        'rpp_params.yaml'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 🔹 Localization (AMCL + map_server)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
            'use_lifecycle_mgr': 'true',
            'autostart': 'true'
        }.items(),
    )

    # 🔹 Navigation (planner + controller + bt)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_lifecycle_mgr': 'true',
            # 'map_subscribe_transient_local': 'true',
            'autostart': 'true'
        }.items(),
    )

    # 🔹 RViz
    rviz_config = os.path.join(
        nav2_bringup_dir,
        'rviz',
        'nav2_default_view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )



    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_params_file,

        localization_launch,
        navigation_launch,

        rviz_node

    ])

