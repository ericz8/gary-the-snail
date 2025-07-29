from setuptools import find_packages, setup

import os
from glob import glob

virtualenv_name = "ardusub"
home_path = os.path.expanduser("~")
executable_path = os.path.join(home_path, '.virtualenvs', virtualenv_name, 'bin', 'python')

package_name = 'gary_the_snail'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devworm',
    maintainer_email='ezhang7708@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_disarm_client = gary_the_snail.bluerov2_arm_disarm:main',
            'depth_hold = gary_the_snail.bluerov2_depth_hold:main',
            'depth_publisher = gary_the_snail.bluerov2_depth_publisher:main',
            'heading_control = gary_the_snail.bluerov2_heading_control:main',
            'target_depth = gary_the_snail.bluerov2_publish_target_depth:main',
            'target_heading = gary_the_snail.bluerov2_publish_target_heading:main',
            'movement_publisher = gary_the_snail.bluerov2_movement:main',
            'control = gary_the_snail.bluerov2_control:main',
            'lane_follow = gary_the_snail.bluerov2_lane_following:main',
            'image_save = gary_the_snail.bluerov2_image_save:main',
            'april_tag = gary_the_snail.bluerov2_april_tag:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': executable_path,
        }
    },
)
