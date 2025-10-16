from setuptools import setup

package_name = 'robotic_cell'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/setup_digital_twin_robotic_cell.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diego',
    maintainer_email='quezadacoloradodiego@gmail.com',
    description='Digital Twin setup for robotic cell',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_pub = robotic_cell.trajectory_pub:main',
            'setup_digital_twin = robotic_cell.setup_digital_twin:main',
            'linear_motor_bridge = robotic_cell.linear_motor_bridge:main',
            'isaac_bridge_lm = robotic_cell.isaac_bridge_lm:main',
            'xarm_gripper_bridge = robotic_cell.xarm_gripper_bridge:main',
            'xarm_linear_motor_rosbridge = robotic_cell.xarm_linear_motor_rosbridge:main',
        ],
    },
)
