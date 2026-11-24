"""
ALT SİMÜLASYON TESTLERİ - tam görev yerine temel becerileri hızlıca dener.

  ros2 launch mrm_bringup skill_test.launch.py                                  # tüm senaryolar
  ros2 launch mrm_bringup skill_test.launch.py scenarios:=pick_basic,pick_place repeat:=2
  ros2 launch mrm_bringup skill_test.launch.py gui:=true rtf:=1.5

Tam sistemden farkları (süre kazancı):
  * PlanSys2, görev kontrolcüsü, RViz yok; tarama ve kapılar atlanır (senaryolar dünya
    modelini doğrudan doldurur, mrm_skills/config/skill_scenarios.yaml)
  * Tüm senaryolar TEK Gazebo oturumunda: robot senaryolar arasında yeniden konumlandırılır
  * Katmanlar 15/30 s yerine 8/12 s'de başlar
  * rtf > 1: fizik gerçek zamandan hızlı koşar (dünya dosyasının hızlandırılmış kopyası)
Test bitince launch kendini kapatır; rapor ~/.ros/mrm_skill_test_report.json
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def inc(pkg, f, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), 'launch', f)),
        launch_arguments=(args or {}).items())


def setup(context):
    lc = LaunchConfiguration
    rtf = float(lc('rtf').perform(context))
    world = os.path.join(get_package_share_directory('mrm_simulation'), 'worlds', 'four_rooms.world')
    if abs(rtf - 1.0) > 1e-3:
        # max_step_size 0.001 s sabit; saniyedeki adım sayısı (update_rate) rtf ile ölçeklenir
        with open(world) as f:
            sdf = f.read()
        sdf = sdf.replace('<real_time_factor>1.0</real_time_factor>',
                          f'<real_time_factor>{rtf}</real_time_factor>')
        sdf = sdf.replace('<real_time_update_rate>1000</real_time_update_rate>',
                          f'<real_time_update_rate>{int(1000 * rtf)}</real_time_update_rate>')
        world = os.path.join(tempfile.gettempdir(), f'mrm_four_rooms_rtf{rtf:g}.world')
        with open(world, 'w') as f:
            f.write(sdf)

    sim = inc('mrm_simulation', 'sim.launch.py', {
        'gui': lc('gui').perform(context), 'world': world, 'pieces_per_room': '0',
        'battery_drain_per_meter': '0.0', 'battery_drain_idle': '0.0'})
    perception = Node(package='mrm_perception', executable='color_detector', output='log',
                      parameters=[{'use_sim_time': True, 'publish_debug_image': False}])
    apriltag = Node(package='apriltag_ros', executable='apriltag_node', name='apriltag',
                    output='log',
                    parameters=[os.path.join(get_package_share_directory('mrm_perception'),
                                             'config', 'apriltag.yaml'), {'use_sim_time': True}],
                    remappings=[('image_rect', '/camera/rgbd/image_raw'),
                                ('camera_info', '/camera/rgbd/camera_info'),
                                ('detections', '/apriltag/detections')])
    skills = Node(package='mrm_skills', executable='skill_server', output='screen',
                  parameters=[{'use_sim_time': True, 'debug_ground_truth': True}])
    tester = Node(package='mrm_skills', executable='skill_test', output='screen',
                  parameters=[{'use_sim_time': True,
                               'scenarios': lc('scenarios').perform(context),
                               'repeat': int(lc('repeat').perform(context)),
                               'report': lc('report').perform(context)}])
    layer2 = TimerAction(period=8.0, actions=[
        inc('mrm_bringup', 'navigation.launch.py'), inc('mrm_bringup', 'moveit.launch.py'),
        perception, apriltag])
    layer3 = TimerAction(period=12.0, actions=[skills, tester])
    done = RegisterEventHandler(OnProcessExit(target_action=tester,
                                              on_exit=[EmitEvent(event=Shutdown(reason='testler bitti'))]))
    return [sim, layer2, layer3, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rtf', default_value='1.0', description='Fizik hizi (gercek zaman carpani)'),
        DeclareLaunchArgument('scenarios', default_value='all', description='virgulle ayrilmis senaryo adlari'),
        DeclareLaunchArgument('repeat', default_value='1'),
        DeclareLaunchArgument('report', default_value=os.path.expanduser('~/.ros/mrm_skill_test_report.json')),
        OpaqueFunction(function=setup),
    ])
