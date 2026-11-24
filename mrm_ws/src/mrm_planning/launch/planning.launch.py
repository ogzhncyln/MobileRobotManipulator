"""PlanSys2 (domain/problem expert, planner, executor) + action performer'lar + beceri sunucusu."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mrm_planning')
    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': os.path.join(pkg, 'pddl', 'mrm_domain.pddl'),
            'params_file': os.path.join(pkg, 'config', 'plansys2_params.yaml'),
            'state_file': '',          # her çalıştırma temiz bilgi tabanıyla başlasın
        }.items())

    performers = Node(package='mrm_planning', executable='action_nodes', output='screen')
    skills = Node(package='mrm_skills', executable='skill_server', output='screen',
                  parameters=[{'use_sim_time': True,
                               'debug_ground_truth': LaunchConfiguration('debug_ground_truth')}])
    mission = Node(package='mrm_planning', executable='mission_controller', output='screen',
                   parameters=[{'use_sim_time': True,
                                'low_battery': LaunchConfiguration('low_battery')}],
                   condition=IfCondition(LaunchConfiguration('start_mission')))

    return LaunchDescription([
        DeclareLaunchArgument('start_mission', default_value='true'),
        DeclareLaunchArgument('low_battery', default_value='30.0'),
        # Sadece teşhis: beceri sunucusu küplerin gerçek konumunu LOGLAR (kontrolde kullanılmaz)
        DeclareLaunchArgument('debug_ground_truth', default_value='false'),
        plansys2, performers, skills,
        TimerAction(period=5.0, actions=[mission]),
    ])
