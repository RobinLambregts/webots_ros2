import os
from setuptools import setup
from glob import glob

package_name = 'webots_ros2_my_robot'
submodule_name = 'my_robot_driver' # Blijft nodig omdat u een submap gebruikt

setup(
    name=package_name,
    version='0.0.1',
    # We installeren de hoofdmap EN de submap
    packages=[package_name, f'{package_name}.{submodule_name}'], 
    # ^-- Dit vertelt setuptools om de map webots_ros2_my_robot en de submap my_robot_driver te installeren.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. LAUNCH FILES: Installeer alle .py bestanden uit de launch map
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.py'))),
        
        # 2. WORLDS: Installeer alle .wbt bestanden uit de worlds map
        (os.path.join('share', package_name, 'worlds'), 
            glob(os.path.join('worlds', '*.wbt'))),
            
        # 3. RESOURCES/URDF: Installeer alle .urdf bestanden uit de resource map
        (os.path.join('share', package_name, 'resource'), 
            glob(os.path.join('resource', '*.urdf'))),
        
        (os.path.join('share', package_name, 'protos'), 
            glob(os.path.join('protos', '*.proto'))),
    ],
    # ... (rest van setup parameters)
    entry_points={
        'console_scripts': [
            # Pad is nu: webots_ros2_my_robot.my_robot_driver.my_robot_driver
            'my_robot_driver = webots_ros2_my_robot.my_robot_driver.my_robot_driver:main',
        ],
    },
)