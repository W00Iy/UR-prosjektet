from glob import glob
from setuptools import find_packages, setup

package_name = 'pick_me'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/srdf', glob('srdf/*')),
        ('share/' + package_name, ['tricoloris.jpg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roko',
    maintainer_email='hoyvik28@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cube_vision_node = pick_me.cube_vision_node:main',
            'cam_to_world_node = pick_me.cam_to_world_node:main',
            'motion_controller_node = pick_me.motion_controller_node:main',
            'main_controller_node = pick_me.color_picker:main',
            'camera_calibration_node = pick_me.camera_calibration_node:main',
            'simple_camera_publisher_node = pick_me.simple_camera_publisher_node:main',
        ],
    },
)


