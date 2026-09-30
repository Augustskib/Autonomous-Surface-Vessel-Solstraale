from setuptools import find_packages, setup

package_name = 'boat_hardware_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'pynmea2', 'smbus2'],
    zip_safe=True,
    maintainer='August Ramberg',
    maintainer_email='augustskib@users.noreply.github.com',
    description='Hardware nodes: Arduino motor driver and IMU relay, GPS, power and system monitoring, simulation thrust bridge.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_publisher = boat_hardware_interface.gps_publisher:main',
            'motor_driver = boat_hardware_interface.motor_driver:main',
            'thrust_splitter = boat_hardware_interface.thrust_splitter:main',
            'imu_publisher = boat_hardware_interface.imu_publisher:main',
            'power_monitor = boat_hardware_interface.power_monitor:main',
            'system_monitor = boat_hardware_interface.system_monitor:main',
        ],
    },
)
