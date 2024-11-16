from setuptools import setup
import os
from glob import glob

package_name = 'unitree_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
            # ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'rviz2'), glob(os.path.join('rviz2', '*.*'))),
        # (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'navigation_to_pose = unitree_nav2.navigation_to_pose:main',
             'navigation_to_pose2 = unitree_nav2.navigation_to_pose2:main',
        ],
    },
)
