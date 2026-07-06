from setuptools import find_packages, setup

package_name = 'visual_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "visual_servo_node = visual_servo.visual_servo_node:main",
            "projection_chain_debug_node = visual_servo.projection_chain_debug_node:main",
            'calibrate_grasp_offset = visual_servo.calibrate_grasp_offset:main',
        ],
    },
)
