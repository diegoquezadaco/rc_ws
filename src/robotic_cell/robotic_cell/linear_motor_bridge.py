import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from xarm.wrapper import XArmAPI
import time

class LinearMotorPublisher(Node):
    def __init__(self):
        super().__init__('linear_motor_publisher')
        self.publisher_ = self.create_publisher(Float64, '/linear_motor/position', 10)
        self.arm = XArmAPI('192.168.1.239')
        self.arm.motion_enable(True)
        self.arm.clean_error()
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        code, pos = self.arm.get_linear_motor_pos()
        if code == 0:
            msg = Float64()
            pos = float(pos)
            msg.data = pos
            self.get_logger().info(f'Publishing linear motor position: {pos}')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LinearMotorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
