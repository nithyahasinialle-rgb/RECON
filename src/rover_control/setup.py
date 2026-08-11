from setuptools import find_packages, setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Research Team',
    maintainer_email='student@research.edu',
    description='Rover teleoperation and ToF obstacle safety brake node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tof_teleop = rover_control.tof_teleop:main',
        ],
    },
)
