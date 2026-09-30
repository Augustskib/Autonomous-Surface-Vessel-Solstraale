from setuptools import find_packages, setup

package_name = 'boat_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='August Ramberg',
    maintainer_email='augustskib@users.noreply.github.com',
    description='Mission state machine, control arbiter and safety supervisor (geofence, link loss, failsafe levels).',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_manager = boat_control.mission_manager:main',
            'safety_monitor = boat_control.safety_monitor:main',
            'control_arbiter = boat_control.control_arbiter:main',
        ],
    },
)
