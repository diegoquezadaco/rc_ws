#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from xarm.wrapper import XArmAPI
import time
import cv2

class LinearMotorPublisher(Node):
    def __init__(self):
        super().__init__('linear_motor_publisher')
        self.publisher_ = self.create_publisher(Float64, '/linear_motor/position', 10)
        self.arm = XArmAPI('192.168.0.239')
        self.arm.motion_enable(True)
        self.arm.clean_error()
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)
        code = self.arm.set_linear_motor_back_origin(wait=True)
        self.get_logger().info(f'Linear motor back to origin, code={code}')

        code, status = self.arm.get_linear_motor_on_zero()
        self.get_logger().info(f'Linear motor on zero status={status}, code={code}')

        code = self.arm.set_linear_motor_enable(True)
        self.get_logger().info(f'Linear motor enable, code={code}')

        code = self.arm.set_linear_motor_speed(400)
        self.get_logger().info(f'Linear motor speed set, code={code}')

        self.arm.set_linear_motor_pos(0, wait=True)
        self.get_logger().info('Linear motor initialized and ready.')

        # Trackbar setup
        self.window_name = "Linear Motor Control"
        cv2.namedWindow(self.window_name)
        self.min_pos = -74
        self.max_pos = 725
        self.slider_max = self.max_pos - self.min_pos
        cv2.createTrackbar('Position', self.window_name, 0, self.slider_max, self.trackbar_callback)
        cv2.setTrackbarPos('Position', self.window_name, 0 - self.min_pos)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def trackbar_callback(self, value):
        target = int(value + self.min_pos)
        self.get_logger().info(f'Moving motor to {target}')
        self.arm.set_linear_motor_pos(target, wait=False)

    def timer_callback(self):
        code, pos = self.arm.get_linear_motor_pos()
        if code == 0:
            msg = Float64()
            pos = float(pos)
            pos_is = (pos * -0.001) + 0.0681
            pos_is = round(pos_is, 3)
            msg.data = pos_is
            self.publisher_.publish(msg)
            self.get_logger().info(f'Isaac_sim position: {pos_is}')
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LinearMotorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.arm.set_linear_motor_stop()
    node.arm.set_linear_motor_enable(False)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
