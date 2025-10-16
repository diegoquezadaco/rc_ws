#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class RealToSimBridge(Node):
    def __init__(self):
        super().__init__('real_to_sim_bridge')

        # Publishers to digital twin
        self.uf850_sim_pub = self.create_publisher(JointState, '/uf850_sim/joint_command', 10)
        self.xarm6_sim_pub = self.create_publisher(JointState, '/xarm6_sim/joint_command', 10)

        # Subscribers to real robot states
        self.uf850_real_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/L_uf850_traj_controller/state',
            self.uf850_real_state_callback,
            10
        )
        self.xarm6_real_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/R_xarm6_traj_controller/state',
            self.xarm6_real_state_callback,
            10
        )

        # Subscribers for gripper positions
        self.gripper_xarm6_real_sub = self.create_subscription(
            Float64,
            '/xarm_gripper_r/position',
            self.gripper_xarm6_real_state_callback,
            10
        )
        self.gripper_uf850_real_sub = self.create_subscription(
            Float64,
            '/xarm_gripper_l/position',
            self.gripper_uf850_real_state_callback,
            10
        )

        # Store latest joint states and gripper values
        self.uf850_last_positions = [0.0] * 7
        self.xarm6_last_positions = [0.0] * 7
        self.uf850_gripper_pos = 0.0
        self.xarm6_gripper_pos = 0.0

        # Joint names
        self.uf850_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'drive_joint']
        self.xarm6_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'drive_joint']

        self.get_logger().info('✅ Real-to-Sim bridge node started')

    def uf850_real_state_callback(self, msg: JointTrajectoryControllerState):
        """Mirror UF850 real robot joints to simulation, adding drive_joint from gripper."""
        positions = list(msg.actual.positions)
        velocities = list(msg.actual.velocities)
        efforts = list(msg.actual.effort)

        # Append gripper position to 7th joint (drive_joint)
        drive_joint_value = self.uf850_gripper_pos
        positions.append(drive_joint_value)
        self.uf850_last_positions = positions

        sim_msg = JointState()
        sim_msg.name = self.uf850_joint_names
        sim_msg.position = positions
        sim_msg.velocity = velocities + [0.0]
        sim_msg.effort = efforts + [0.0]

        self.uf850_sim_pub.publish(sim_msg)
        self.get_logger().info(f'Mirrored UF850 → sim: {positions}')

    def xarm6_real_state_callback(self, msg: JointTrajectoryControllerState):
        """Mirror XArm6 real robot joints to simulation, adding drive_joint from gripper."""
        positions = list(msg.actual.positions)
        velocities = list(msg.actual.velocities)
        efforts = list(msg.actual.effort)

        drive_joint_value = self.xarm6_gripper_pos
        positions.append(drive_joint_value)
        self.xarm6_last_positions = positions

        sim_msg = JointState()
        sim_msg.name = self.xarm6_joint_names
        sim_msg.position = positions
        sim_msg.velocity = velocities + [0.0]
        sim_msg.effort = efforts + [0.0]

        self.xarm6_sim_pub.publish(sim_msg)
        self.get_logger().info(f'Mirrored XArm6 → sim: {positions}')

    def gripper_uf850_real_state_callback(self, msg: Float64):
        """Update left gripper position value."""
        self.uf850_gripper_pos = msg.data
        self.get_logger().info(f'UF850 gripper updated: {msg.data}')

    def gripper_xarm6_real_state_callback(self, msg: Float64):
        """Update right gripper position value."""
        self.xarm6_gripper_pos = msg.data
        self.get_logger().info(f'XArm6 gripper updated: {msg.data}')



def main(args=None):
    rclpy.init(args=args)
    node = RealToSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
