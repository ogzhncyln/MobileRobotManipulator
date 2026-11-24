from glob import glob

from setuptools import setup

package_name = 'mrm_skills'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True, tests_require=['pytest'],
    maintainer='MRM Developer', maintainer_email='dev@example.com',
    description='Nav2 + MoveIt + algilama ile robot becerileri (beceri sunucusu)', license='Apache-2.0',
    entry_points={'console_scripts': ['skill_server = mrm_skills.skill_server:main',
                                      'skill_test = mrm_skills.skill_test:main']},
)
