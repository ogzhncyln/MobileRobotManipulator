"""
Nav2: map_server + AMCL (lokalizasyon) + planner/controller/behavior/bt_navigator.

İKİ HARİTA kullanılır:
  /map      four_rooms_amcl.yaml  kapılar KAPALI  -> AMCL (lidar kapalı kapıyı görür; harita uyumlu)
  /map_nav  four_rooms.yaml       kapılar AÇIK    -> global costmap static layer
            (planlayıcı kapıdan geçen yol üretebilsin; kapalı kapıyı lidar engel katmanı işaretler)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('mrm_bringup')
    sim = get_package_share_directory('mrm_simulation')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': os.path.join(sim, 'maps', 'four_rooms_amcl.yaml'),
            'params_file': os.path.join(bringup, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true', 'autostart': 'true', 'slam': 'False',
            'use_composition': 'False',
        }.items())
    map_nav = Node(
        package='nav2_map_server', executable='map_server', name='map_server_nav',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(sim, 'maps', 'four_rooms.yaml'),
                     'topic_name': 'map_nav', 'frame_id': 'map', 'use_sim_time': True}])
    map_nav_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map_nav', output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server_nav'], 'use_sim_time': True}])
    return LaunchDescription([nav2, map_nav, map_nav_lifecycle])
