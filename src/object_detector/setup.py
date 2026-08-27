import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dfki',
    maintainer_email='dfki@example.com',
    description='YOLO object detector for TurtleBot 4.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detect = object_detector.detect:main',
            'detect_3d = object_detector.detect_3d:main',
            'search = object_detector.search_runner:main',
            'mission = object_detector.mission:main',
            'object_map = object_detector.object_map:main',
        ],
    },
)