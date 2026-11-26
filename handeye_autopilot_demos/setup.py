from setuptools import find_packages, setup

package_name = 'handeye_autopilot_demos'

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
    description='Panda and UR mock-hardware demos for handeye_autopilot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
