from setuptools import setup

import os
from glob import glob

package_name = 'pcd_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # During installation, we need to copy the launch files
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        # Same with the RViz configuration file.
        (os.path.join('share', package_name, "config"), glob('config/*')),
        # And the ply files.
        (os.path.join('share', package_name, "resource"), glob('resource/*.ply')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Grans',
    maintainer_email='sebastian.grans@ntno.com',
    description='Demo package on how to visualize a point cloud using RViz',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcd_publisher_node = pcd_demo.pcd_publisher.pcd_publisher_node:main',
            'pcd_subscriber_node = pcd_demo.pcd_subscriber.pcd_subscriber_node:main'
        ],
    },
)
