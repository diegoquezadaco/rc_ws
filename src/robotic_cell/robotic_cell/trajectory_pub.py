import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_pub')
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.points = [
            [-1.209805264491853,-0.59362653361019067,-0.8231073266696219,
                0.6250106349174553, -1.174041508970718,-2.902024968019263],
            [-0.809805264491853, -0.29362653361019067, -0.8231073266696219,
                0.6250106349174553, -1.174041508970718, -2.702024968019263],
            [-0.62767531118298396,-0.7076362764227252,-1.4695435533552117,
                0.38265232906167784, -1.4424422621716392, -2.70912643877071494],
            [0.0242152105086917, -0.7524486466927167, -1.4092815781344825,
                -0.0269193581254857, -1.4938572536138504, -3.079542059666135917],
            [0.5242152105086917, -0.6524486466927167, -1.4092815781344825,
                -0.1269193581254857, -1.4938572536138504, -3.279542059666135917],
            [0.7233068447773152, -0.70034055297091893, -1.2608655227106162,
                -0.2955318732328511, -1.5217809184135878, -3.3098725647896004],
            [0.9233068447773152, -0.71934055297091893, -1.1608655227106162,
                -0.4955318732328511, -1.5417809184135878, -3.4098725647896004],
            [1.233068447773152, -0.81934055297091893, -1.0608655227106162,
                -0.4955318732328511, -1.5417809184135878, -3.29098725647896004],
            [1.9233068447773152, -0.91934055297091893, -0.8608655227106162,
                -0.4955318732328511, -1.5417809184135878, -3.09098725647896004]
        ]
        self.current_idx = 0
        self.tolerance = 0.01
        self.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        self.interp_step = 0.1  # interpolation step size (0 < interp_step <= 1)
        self.last_sent_position = None

    def joint_state_callback(self, msg):
        if self.current_idx >= len(self.points):
            return  # All points sent

        # Get current joint positions in correct order
        current_positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                current_positions.append(msg.position[idx])
            else:
                return  # Wait for all joints to be present

        target = np.array(self.points[self.current_idx])
        current = np.array(current_positions)

        if np.all(np.abs(current - target) < self.tolerance):
            # Arrived at target, move to next
            self.current_idx += 1
            self.last_sent_position = None
            return

        # Interpolate towards target
        if self.last_sent_position is None:
            start = current
        else:
            start = self.last_sent_position

        direction = target - start
        distance = np.linalg.norm(direction)
        if distance < self.tolerance:
            interp = target
        else:
            interp = start + self.interp_step * direction

        self.last_sent_position = interp

        cmd = JointState()
        cmd.name = self.joint_names
        cmd.position = interp.tolist()
        cmd.velocity = [0.0]*6
        self.pub.publish(cmd)
        self.get_logger().info(f'Interpolating to point {self.current_idx + 1}: {interp}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
