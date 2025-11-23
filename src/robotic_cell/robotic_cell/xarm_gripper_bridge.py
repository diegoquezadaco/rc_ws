#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from xarm.wrapper import XArmAPI
import time
import cv2
import numpy as np

class XArmGripperPublisher(Node):
    def __init__(self):
        super().__init__('xarm_gripper_publisher')

        # Publishers
        self.publisher_l = self.create_publisher(Float64, '/xarm_gripper_l/position', 10)
        self.publisher_r = self.create_publisher(Float64, '/xarm_gripper_r/position', 10)

        # ===== LEFT GRIPPER (UF850) =====
        self.arm_l = XArmAPI('192.168.0.211')
        self.arm_l.motion_enable(True)
        self.arm_l.clean_error()
        self.arm_l.set_mode(0)
        self.arm_l.set_state(0)
        time.sleep(1)
        self.arm_l.set_gripper_mode(0)
        self.arm_l.set_gripper_enable(True)
        self.arm_l.set_gripper_speed(5000)
        self.arm_l.set_gripper_position(839, wait=True)
        self.get_logger().info('Left gripper initialized and ready.')

        # ===== RIGHT GRIPPER (XArm6) =====
        self.arm_r = XArmAPI('192.168.0.239')
        self.arm_r.motion_enable(True)
        self.arm_r.clean_error()
        self.arm_r.set_mode(0)
        self.arm_r.set_state(0)
        time.sleep(1)
        self.arm_r.set_gripper_mode(0)
        self.arm_r.set_gripper_enable(True)
        self.arm_r.set_gripper_speed(5000)
        self.arm_r.set_gripper_position(839, wait=True)
        self.get_logger().info('Right gripper initialized and ready.')

        # ===== Trackbar setup =====
        self.window_name = "Dual Gripper Control"
        cv2.namedWindow(self.window_name)
        self.min_pos = 0
        self.max_pos = 839

        cv2.createTrackbar('UF850 (Left Gripper)', self.window_name, 839, self.max_pos, self.on_left_change)
        cv2.createTrackbar('XArm6 (Right Gripper)', self.window_name, 839, self.max_pos, self.on_right_change)

        self.pos_l = 839
        self.pos_r = 839

        self.timer = self.create_timer(0.5, self.timer_callback)

    def on_left_change(self, value):
        self.pos_l = int(value)
        self.arm_l.set_gripper_position(self.pos_l, wait=True)

    def on_right_change(self, value):
        self.pos_r = int(value)
        self.arm_r.set_gripper_position(self.pos_r, wait=True)

    def timer_callback(self):
        # Publish left position
        l = self.pos_l * -0.001 + 0.85
        r = self.pos_r * -0.001 + 0.85
        l = round(l, 3)
        r = round(r, 3)

        msg_l = Float64()
        msg_l.data = float(l)
        self.publisher_l.publish(msg_l)

        # Publish right position
        msg_r = Float64()
        msg_r.data = float(r)
        self.publisher_r.publish(msg_r)

        self.get_logger().info(f'Published positions - Left: {l}, Right: {r}')

        #cv2.imshow(self.window_name, 255 * np.ones((100, 400, 3), dtype=np.uint8))
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = XArmGripperPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm_l.set_gripper_enable(False)
        node.arm_r.set_gripper_enable(False)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    import numpy as np
    main()
