import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'frame_grabber'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files so `ros2 launch frame_grabber ...` can find them.
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dfki',
    maintainer_email='dfki@example.com',
    description='Save a rolling buffer of OAK-D camera frames to disk.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grab_frames = frame_grabber.grab_frames:main',
        ],
    },
)