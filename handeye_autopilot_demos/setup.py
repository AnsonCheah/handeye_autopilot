from glob import glob

from setuptools import find_packages, setup

package_name = 'handeye_autopilot_demos'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anson Cheah',
    maintainer_email='ansoncta.sg@gmail.com',
    description='Panda and UR mock-hardware demos for handeye_autopilot.',
    license='Apache-2.0',
    # colcon runs pytest when it is listed here (tests_require is gone in new setuptools)
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'scene = handeye_autopilot_demos.scene:main',
        'run_demo = handeye_autopilot_demos.run_demo:main',
        'sweep_mounts = handeye_autopilot_demos.sweep_mounts:main',
    ]},
)
