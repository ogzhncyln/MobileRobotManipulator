"""
MoveIt 2 move_group düğümü.
Robot modeli bizim URDF'imiz (mrm_description); anlamsal model (SRDF), kinematik,
OMPL ve kontrolcü yapılandırmaları upstream turtlebot3_manipulation_moveit_config'ten.
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    mc = 'turtlebot3_manipulation_moveit_config'
    urdf = xacro.process_file(os.path.join(
        get_package_share_directory('mrm_description'), 'urdf', 'mrm_robot.urdf.xacro'),
        mappings={'use_sim': 'true'}).toxml()
    with open(os.path.join(get_package_share_directory(mc), 'config',
                           'turtlebot3_manipulation.srdf')) as f:
        srdf = f.read()

    ompl = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': ('default_planner_request_adapters/AddTimeOptimalParameterization '
                             'default_planner_request_adapters/FixWorkspaceBounds '
                             'default_planner_request_adapters/FixStartStateBounds '
                             'default_planner_request_adapters/FixStartStateCollision '
                             'default_planner_request_adapters/FixStartStatePathConstraints'),
        'start_state_max_bounds_error': 0.1}}
    ompl['move_group'].update(load_yaml(mc, 'config/ompl_planning.yaml'))

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[
            {'robot_description': urdf},
            {'robot_description_semantic': srdf},
            {'robot_description_kinematics': load_yaml(mc, 'config/kinematics.yaml')},
            {'robot_description_planning': load_yaml(mc, 'config/joint_limits.yaml')},
            ompl,
            {'moveit_manage_controllers': True,
             'trajectory_execution.allowed_execution_duration_scaling': 1.5,
             'trajectory_execution.allowed_goal_duration_margin': 1.0,
             'trajectory_execution.allowed_start_tolerance': 0.05},
            {'moveit_simple_controller_manager': load_yaml(mc, 'config/moveit_controllers.yaml'),
             'moveit_controller_manager':
                 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'publish_planning_scene': True, 'publish_geometry_updates': True,
             'publish_state_updates': True, 'publish_transforms_updates': True},
            {'use_sim_time': True},
        ])
    return LaunchDescription([move_group])
