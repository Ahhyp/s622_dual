from setuptools import find_packages, setup

package_name = 'yolov8_grasping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/demo_node.yaml',
            'config/demo_node_without_gripper.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/demo.launch.py',
            'launch/demo_without_gripper.launch.py',
        ]),
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
            'grasping_node = yolov8_grasping.grasping_node:main',
            "grasp_executor_node = yolov8_grasping.executor_node:main",
            "demo_node = yolov8_grasping.demo_node:main",
            "demo_node_without_gripper = yolov8_grasping.demo_node_without_gripper:main",
        ],
    },
)
