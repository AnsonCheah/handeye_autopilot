from setuptools import setup

setup(
    name='interface_consumer',
    version='0.0.0',
    packages=['interface_consumer'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/interface_consumer']),
        ('share/interface_consumer', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='Apache-2.0',
    tests_require=['pytest'],
)
