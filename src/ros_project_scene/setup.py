from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_project_scene'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='likerobotics',
    maintainer_email='xmlpro100@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_publisher = ros_project_scene.scene_publisher:main',
            'sensor_data_proc = ros_project_scene.sensor_data_proc:main',
            'lidar_data_proc = ros_project_scene.lidar_data_proc:main',
            'controller = ros_project_scene.controller:main',
        ],
    },
)
