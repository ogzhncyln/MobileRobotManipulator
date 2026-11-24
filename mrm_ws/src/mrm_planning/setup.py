import os
from glob import glob

from setuptools import setup

package_name = 'mrm_planning'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'pddl'), glob('pddl/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='MRM Developer', maintainer_email='dev@example.com',
    description='PlanSys2 domain, action performerlar ve gorev kontrolcusu', license='Apache-2.0',
    entry_points={'console_scripts': [
        'action_nodes = mrm_planning.action_nodes:main',
        'mission_controller = mrm_planning.mission_controller:main',
    ]},
)
