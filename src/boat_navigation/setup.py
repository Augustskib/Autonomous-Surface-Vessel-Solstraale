from glob import glob
from setuptools import find_packages, setup

package_name = 'boat_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='August Ramberg',
    maintainer_email='augustskib@users.noreply.github.com',
    description='Heading estimation (gyro and GPS course fusion) and GPS waypoint following.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_follower = boat_navigation.waypoint_follower:main',
            'fusion_inputs = boat_navigation.fusion_inputs:main',
            'heading_estimator = boat_navigation.heading_estimator:main',
        ],
    },
)
