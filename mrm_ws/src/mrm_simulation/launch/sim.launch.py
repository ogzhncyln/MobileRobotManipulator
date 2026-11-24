"""
Simülasyon katmanı: Gazebo + robot + kontrolcüler + rastgele parçalar + batarya.

  ros2 launch mrm_simulation sim.launch.py gui:=true pieces_per_room:=3 seed:=-1
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('mrm_simulation')
    desc_share = get_package_share_directory('mrm_description')
    world_file = os.path.join(sim_share, 'worlds', 'four_rooms.world')
    with open(os.path.join(sim_share, 'config', 'world.yaml')) as f:
        sx, sy, syaw = yaml.safe_load(f)['robot_start']

    gui = LaunchConfiguration('gui')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc_share, 'urdf', 'mrm_robot.urdf.xacro'), ' use_sim:=true']),
        value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'gui': gui, 'verbose': 'false'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'mrm_robot',
                   '-x', str(sx), '-y', str(sy), '-z', '0.01', '-Y', str(syaw)])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60'])

    jsb = spawner('joint_state_broadcaster')
    controllers_after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=[jsb]))
    arm_after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=jsb, on_exit=[spawner('arm_controller'), spawner('gripper_controller'),
                 spawner('gripper_right_controller')]))

    pieces = TimerAction(period=8.0, actions=[Node(
        package='mrm_simulation', executable='piece_spawner', output='screen',
        parameters=[{'pieces_per_room': LaunchConfiguration('pieces_per_room'),
                     'seed': LaunchConfiguration('seed'), 'use_sim_time': True}])])

    battery = Node(package='mrm_simulation', executable='battery_sim', output='screen',
                   parameters=[{'use_sim_time': True,
                                'initial_level': LaunchConfiguration('battery_initial'),
                                'drain_per_meter': LaunchConfiguration('battery_drain_per_meter'),
                                'drain_idle': LaunchConfiguration('battery_drain_idle')}])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo arayuzu'),
        DeclareLaunchArgument('world', default_value=world_file,
                              description='SDF dunya dosyasi (alt simulasyon testleri hizlandirilmis kopya verir)'),
        DeclareLaunchArgument('pieces_per_room', default_value='3'),
        DeclareLaunchArgument('seed', default_value='-1', description='-1: rastgele'),
        DeclareLaunchArgument('battery_initial', default_value='100.0'),
        DeclareLaunchArgument('battery_drain_per_meter', default_value='0.8'),
        DeclareLaunchArgument('battery_drain_idle', default_value='0.04'),
        # Gazebo'nun iç iletişimi (gzserver <-> gzclient, gz araçları) LOOPBACK'e sabitlenir.
        # GAZEBO_IP boşken Gazebo makinenin ağ IP'sine (ör. Wi-Fi) bağlanır; Wi-Fi koptuğunda
        # "Exception sending a multicast message: Network is unreachable" ile Gazebo penceresi
        # donar, zaman durmuş görünür (fizik ve ROS ise çalışmaya devam eder).
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345'),
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
        # package:// mesh yollarının Gazebo tarafından bulunabilmesi için
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(sim_share, 'models') + ':/opt/ros/humble/share:'
                               + os.environ.get('GAZEBO_MODEL_PATH', '') + ':/usr/share/gazebo-11/models'),
        gazebo, rsp, spawn_robot, controllers_after_spawn, arm_after_jsb, pieces, battery,
    ])
