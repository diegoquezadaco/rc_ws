#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState


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

        self.uf850_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.xarm6_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        self.get_logger().info('✅ Real-to-Sim bridge node started')

    def uf850_real_state_callback(self, msg: JointTrajectoryControllerState):
        """Mirror the UF850 real robot’s joint positions into the simulation."""
        sim_msg = JointState()
        sim_msg.name = self.uf850_joint_names
        sim_msg.position = list(msg.actual.positions)
        sim_msg.velocity = list(msg.actual.velocities)
        sim_msg.effort = list(msg.actual.effort)

        self.uf850_sim_pub.publish(sim_msg)
        self.get_logger().info(f'Mirrored UF850 real → sim: {sim_msg.position}')

    def xarm6_real_state_callback(self, msg: JointTrajectoryControllerState):
        """Mirror the xArm6 real robot’s joint positions into the simulation."""
        sim_msg = JointState()
        sim_msg.name = self.xarm6_joint_names
        sim_msg.position = list(msg.actual.positions)
        sim_msg.velocity = list(msg.actual.velocities)
        sim_msg.effort = list(msg.actual.effort)

        self.xarm6_sim_pub.publish(sim_msg)
        self.get_logger().info(f'Mirrored xArm6 real → sim: {sim_msg.position}')


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
