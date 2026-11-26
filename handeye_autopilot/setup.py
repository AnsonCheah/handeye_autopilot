from setuptools import find_packages, setup

package_name = 'handeye_autopilot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anson Cheah',
    maintainer_email='ansoncta.sg@gmail.com',
    description='Automated, quality-gated hand-eye calibration for ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
