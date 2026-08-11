from setuptools import find_packages, setup

package_name = 'tof_sensor_sim'

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
    description='4x ST VL53L0X ToF distance sensor publisher node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tof_publisher_node = tof_sensor_sim.tof_publisher_node:main',
        ],
    },
)
