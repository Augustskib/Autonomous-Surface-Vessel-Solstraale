from setuptools import find_packages, setup

package_name = 'boat_data'

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
    description='Mission-aware rosbag2 (MCAP) recorder with a run archive and write monitoring.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'data_recorder = boat_data.data_recorder:main',
        ],
    },
)
