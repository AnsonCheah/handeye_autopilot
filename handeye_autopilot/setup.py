from glob import glob

from setuptools import find_packages, setup

package_name = 'handeye_autopilot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anson Cheah',
    maintainer_email='ansoncta.sg@gmail.com',
    description='Automated, quality-gated hand-eye calibration for ROS 2.',
    license='Apache-2.0',
    # colcon runs pytest when it is listed here (tests_require is gone in new setuptools)
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'board_image = handeye_autopilot.cli.board_image:main',
        'resolve = handeye_autopilot.cli.resolve:main',
        'calibrate_node = handeye_autopilot.ros.node:main',
        'synthetic_camera = handeye_autopilot.ros.synthetic_camera:main',
        'publish_result = handeye_autopilot.ros.publish_result:main',
    ]},
)
