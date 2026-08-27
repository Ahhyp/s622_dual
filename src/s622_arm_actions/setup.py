from setuptools import find_packages, setup
import os
from glob import glob

package_name = 's622_arm_actions'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yep',
    maintainer_email='1056651817@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_pose_server = s622_arm_actions.move_to_pose_server:main',
            'gripper_service = s622_arm_actions.gripper_service:main',
            'visual_align_server = s622_arm_actions.visual_align_server:main',
            'planning_scene_service = s622_arm_actions.planning_scene_service:main',
            'dual_move_server = s622_arm_actions.dual_move_server:main',
            'nan_diagnose = s622_arm_actions.nan_diagnose:main',
            'demo_node_without_gripper = s622_arm_actions.demo_node_without_gripper:main',
        ],
    },
)
