from setuptools import setup

package_name = 'mrm_perception'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/config', ['config/apriltag.yaml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='MRM Developer', maintainer_email='dev@example.com',
    description='RGB-D + HSV renkli kup algilayici', license='Apache-2.0',
    entry_points={'console_scripts': ['color_detector = mrm_perception.color_detector:main']},
)
