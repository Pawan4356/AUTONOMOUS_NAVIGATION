import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models', 'world'), glob('models/world/*.sdf')),
        (os.path.join('share', package_name, 'models', 'world', 'city'), glob('models/world/city/*.sdf')),
        (os.path.join('share', package_name, 'models', 'auto_vehicle'), glob('models/auto_vehicle/*.xacro')),
        (os.path.join('share', package_name, 'models', 'auto_vehicle'), glob('models/auto_vehicle/*.gazebo')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pawan',
    maintainer_email='pawankumar27112005@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
