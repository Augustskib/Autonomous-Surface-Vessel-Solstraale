from setuptools import find_packages, setup

package_name = 'boat_web_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['operator_controls.js']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='August Ramberg',
    maintainer_email='augustskib@users.noreply.github.com',
    description='Web control station: map, manual driving, missions, failsafe banner and data collection',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_server = boat_web_ui.web_server:main',
        ],
    },
)
