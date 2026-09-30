from setuptools import find_packages, setup

package_name = 'boat_perception'

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
    description='Camera publisher, YOLO detector (disabled by default) and ArUco dock marker detection.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = boat_perception.camera_publisher:main',
            'detector_node = boat_perception.detector_node:main',
            'dock_marker_node = boat_perception.dock_marker_node:main',
        ],
    },
)
