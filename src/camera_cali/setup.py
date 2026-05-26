import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_cali'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wooly',
    maintainer_email='wooly@todo.todo',
    description='Camera calibration package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'getImages = camera_cali.getImages:main',
            'calibrate = camera_cali.calibrate:main',
            'move_and_calibrate = camera_cali.move_and_calibrate:main',
        ],
    },
)