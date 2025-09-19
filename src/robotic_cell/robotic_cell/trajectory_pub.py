import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import numpy as np

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_pub')
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.start_time = time.time()

        # Trajectory points: (time_from_start, [joint positions])
        self.points = [
            (10,  [-1.209805264491853,-0.59362653361019067,-0.8231073266696219,
                    0.6250106349174553, -1.174041508970718,-2.902024968019263]),
            (20, [-0.809805264491853, -0.29362653361019067, -0.8231073266696219,
                    0.6250106349174553, -1.174041508970718, -2.702024968019263]),
            (30, [-0.62767531118298396,-0.7076362764227252,-1.4695435533552117,
                    0.38265232906167784, -1.4424422621716392, -2.70912643877071494]),
            (50, [0.0242152105086917, -0.7524486466927167, -1.4092815781344825,
                    -0.2269193581254857, -1.4938572536138504, -3.079542059666135917]),
            (65, [0.5242152105086917, -0.6524486466927167, -1.4092815781344825,
                    -0.1269193581254857, -1.4938572536138504, -3.279542059666135917]),
            (85, [0.9233068447773152, -0.71934055297091893, -1.1608655227106162,
                    -0.4955318732328511, -1.5417809184135878, -3.4098725647896004]),
            (100, [1.9233068447773152, -0.91934055297091893, -0.8608655227106162,
                    -0.4955318732328511, -1.5417809184135878, -3.09098725647896004])
        ]

        self.current_idx = 0

    def timer_callback(self):
        now = time.time() - self.start_time

        # stop after last point
        if self.current_idx >= len(self.points) - 1:
            return

        t0, p0 = self.points[self.current_idx]
        t1, p1 = self.points[self.current_idx + 1]

        if now < t0:
            return  # wait until first point

        if now >= t1:
            # reached next point, move index forward
            self.current_idx += 1
            return

        # --- Linear interpolation ---
        ratio = (now - t0) / (t1 - t0)
        p0 = np.array(p0)
        p1 = np.array(p1)
        interp = (1 - ratio) * p0 + ratio * p1

        msg = JointState()
        msg.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
        msg.position = interp.tolist()
        msg.velocity = [0.0]*6
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
