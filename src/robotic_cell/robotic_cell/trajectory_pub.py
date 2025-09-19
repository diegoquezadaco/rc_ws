import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_pub')
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.start_time = time.time()

        # Trajectory points: (time_from_start, [joint positions])
        self.points = [
            (5,  [-0.6098, -0.1936, -0.8231, 0.6250, -1.1740, 1.0202]),
            (10, [-0.4277, -0.7076, -1.4695, 0.3826, -1.8424, 0.0912]),
            (15, [0.5242, -0.6524, -1.4092, -0.1269, -1.4938, -0.2954]),
            (20, [0.9233, -0.4193, -1.1608, -0.4955, -1.5417, -0.4098])
        ]
        self.current_idx = 0

    def timer_callback(self):
        now = time.time() - self.start_time
        if self.current_idx < len(self.points) and now >= self.points[self.current_idx][0]:
            msg = JointState()
            msg.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
            msg.position = self.points[self.current_idx][1]
            msg.velocity = [0.0]*6
            self.pub.publish(msg)
            self.get_logger().info(f"Sent point {self.current_idx}")
            self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
