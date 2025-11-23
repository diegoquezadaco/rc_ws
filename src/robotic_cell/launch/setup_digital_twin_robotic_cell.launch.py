from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1️⃣ Run the xarm linear motor bridge
        Node(
            package='robotic_cell',
            executable='xarm_linear_motor_rosbridge',
            name='xarm_linear_motor_rosbridge',
            output='screen'
        ),

        # 2️⃣ Run the isaac bridge for linear motor
        Node(
            package='robotic_cell',
            executable='isaac_bridge_lm',
            name='isaac_bridge_lm',
            output='screen'
        ),

        # 3️⃣ Run the setup digital twin node
        Node(
            package='robotic_cell',
            executable='setup_digital_twin',
            name='setup_digital_twin',
            output='screen'
        )
    ])
