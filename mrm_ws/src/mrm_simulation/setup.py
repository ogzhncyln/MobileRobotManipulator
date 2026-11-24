import os
from glob import glob

from setuptools import setup

package_name = 'mrm_simulation'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        # AprilTag dokuları (Gazebo model://mrm_apriltags/...)
        (os.path.join('share', package_name, 'models', 'mrm_apriltags'),
         glob('models/mrm_apriltags/model.*')),
        (os.path.join('share', package_name, 'models', 'mrm_apriltags', 'materials', 'scripts'),
         glob('models/mrm_apriltags/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'mrm_apriltags', 'materials', 'textures'),
         glob('models/mrm_apriltags/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MRM Developer',
    maintainer_email='dev@example.com',
    description='Dort odali Gazebo dunyasi, parca uretici ve batarya simulatoru',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'piece_spawner = mrm_simulation.piece_spawner:main',
        'battery_sim = mrm_simulation.battery_sim:main',
    ]},
)
