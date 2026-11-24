"""
TÜM SİSTEM - tek komut:
  ros2 launch mrm_bringup full_system.launch.py

Katmanlar ve başlama sırası:
  t=0   Simülasyon (Gazebo, robot, kontrolcüler, parçalar, batarya)
  t=15  Nav2 (harita + AMCL + navigasyon) ve MoveIt (move_group), algılama, RViz
  t=30  PlanSys2 + action performer'lar + beceri sunucusu + görev kontrolcüsü
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def inc(pkg, f, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), 'launch', f)),
        launch_arguments=(args or {}).items())


def generate_launch_description():
    bringup = get_package_share_directory('mrm_bringup')
    lc = LaunchConfiguration
    sim = inc('mrm_simulation', 'sim.launch.py', {
        'gui': lc('gui'), 'pieces_per_room': lc('pieces_per_room'), 'seed': lc('seed'),
        'battery_initial': lc('battery_initial'),
        'battery_drain_per_meter': lc('battery_drain_per_meter')})
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(bringup, 'rviz', 'mrm.rviz')],
                parameters=[{'use_sim_time': True}], condition=IfCondition(lc('rviz')))
    perception = Node(package='mrm_perception', executable='color_detector', output='screen',
                      parameters=[{'use_sim_time': True}])
    # AprilTag algılayıcı: kutu ve dock etiketlerinin kameraya göre pozunu TF olarak yayınlar
    # (rgbd_camera_optical_frame -> tag_<id>)
    apriltag = Node(package='apriltag_ros', executable='apriltag_node', name='apriltag',
                    output='log',
                    parameters=[os.path.join(get_package_share_directory('mrm_perception'),
                                             'config', 'apriltag.yaml'), {'use_sim_time': True}],
                    remappings=[('image_rect', '/camera/rgbd/image_raw'),
                                ('camera_info', '/camera/rgbd/camera_info'),
                                ('detections', '/apriltag/detections')])
    layer2 = TimerAction(period=15.0, actions=[
        inc('mrm_bringup', 'navigation.launch.py'),
        inc('mrm_bringup', 'moveit.launch.py'),
        perception, apriltag, rviz])
    layer3 = TimerAction(period=30.0, actions=[
        inc('mrm_planning', 'planning.launch.py', {
            'start_mission': lc('start_mission'), 'low_battery': lc('low_battery'),
            'debug_ground_truth': lc('debug_ground_truth')})])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('pieces_per_room', default_value='3'),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('start_mission', default_value='true'),
        DeclareLaunchArgument('low_battery', default_value='30.0'),
        DeclareLaunchArgument('debug_ground_truth', default_value='false'),
        DeclareLaunchArgument('battery_initial', default_value='100.0'),
        DeclareLaunchArgument('battery_drain_per_meter', default_value='0.8'),
        sim, layer2, layer3,
    ])
