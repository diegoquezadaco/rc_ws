#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState


class RealToSimBridge(Node):
    def __init__(self):
        super().__init__('real_to_sim_bridge')

        # Publisher to digital twin
        #self.sim_pub = self.create_publisher(JointState, '/uf850_sim/joint_command', 10)
        self.sim_pub = self.create_publisher(JointState, '/xarm6_sim/joint_command', 10)
        # Subscriber to real robot state
        # self.real_sub = self.create_subscription(
        #     JointTrajectoryControllerState,
        #     '/uf850_traj_controller/state',
        #     self.real_state_callback,
        #     10
        # )
        self.real_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/xarm6_traj_controller/state',
            self.real_state_callback,
            10
        )


        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.sim_last_state = None
        self.get_logger().info('✅ Real-to-Sim bridge node started')

    def real_state_callback(self, msg: JointTrajectoryControllerState):
        """Mirror the real robot’s joint positions into the simulation."""
        sim_msg = JointState()
        sim_msg.name = self.joint_names
        sim_msg.position = list(msg.actual.positions)
        sim_msg.velocity = list(msg.actual.velocities)
        sim_msg.effort = list(msg.actual.effort)

        self.sim_pub.publish(sim_msg)
        self.get_logger().info(f'Mirrored real → sim: {sim_msg.position}')

    def sim_state_callback(self, msg: JointState):
        """Optionally track simulation state for later use (e.g., feedback)."""
        self.sim_last_state = msg.position


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
