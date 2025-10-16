#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from xarm.wrapper import XArmAPI
import time
import cv2
import numpy as np


class MultiDevicePublisher(Node):
    def __init__(self):
        super().__init__('multi_device_publisher')

        # === Publishers ===
        self.publisher_linear = self.create_publisher(Float64, '/linear_motor/position', 10)
        self.publisher_gripper_l = self.create_publisher(Float64, '/xarm_gripper_l/position', 10)
        self.publisher_gripper_r = self.create_publisher(Float64, '/xarm_gripper_r/position', 10)

        # === Initialize Linear Motor (XArm6) ===
        self.arm_r = XArmAPI('192.168.0.239')
        self.arm_r.motion_enable(True)
        self.arm_r.clean_error()
        self.arm_r.set_mode(0)
        self.arm_r.set_state(0)
        time.sleep(1)
        code = self.arm_r.set_linear_motor_back_origin(wait=True)
        self.get_logger().info(f'Linear motor back to origin, code={code}')

        code = self.arm_r.set_linear_motor_enable(True)
        self.get_logger().info(f'Linear motor enable, code={code}')

        code = self.arm_r.set_linear_motor_speed(100)
        self.get_logger().info(f'Linear motor speed set, code={code}')

        self.arm_r.set_linear_motor_pos(0, wait=True)
        self.get_logger().info('Linear motor initialized and ready.')

        # === Initialize Grippers ===
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
        self.get_logger().info('Left gripper (UF850) initialized.')

        time.sleep(1)
        self.arm_r.set_gripper_mode(0)
        self.arm_r.set_gripper_enable(True)
        self.arm_r.set_gripper_speed(5000)
        self.arm_r.set_gripper_position(839, wait=True)
        self.get_logger().info('Right gripper (XArm6) initialized.')

        # === Trackbar setup ===
        self.window_name = "Linear Motor + Dual Gripper Control"
        cv2.namedWindow(self.window_name)

        # Linear motor limits
        self.linear_min = -74
        self.linear_max = 725
        self.linear_slider_max = self.linear_max - self.linear_min
        cv2.createTrackbar('Linear Motor', self.window_name, 0, self.linear_slider_max, self.trackbar_linear)
        cv2.setTrackbarPos('Linear Motor', self.window_name, 0 - self.linear_min)

        # Gripper limits
        self.gripper_min = 0
        self.gripper_max = 839
        cv2.createTrackbar('UF850 (Left Gripper)', self.window_name, 839, self.gripper_max, self.trackbar_left)
        cv2.createTrackbar('XArm6 (Right Gripper)', self.window_name, 839, self.gripper_max, self.trackbar_right)

        # === State variables ===
        self.linear_target = 0
        self.gripper_l_target = 839
        self.gripper_r_target = 839

        # === Timer for publishing ===
        self.timer = self.create_timer(0.1, self.timer_callback)

    # === Trackbar callbacks ===
    def trackbar_linear(self, value):
        self.linear_target = int(value + self.linear_min)
        self.get_logger().info(f'Moving linear motor to {self.linear_target}')
        self.arm_r.set_linear_motor_pos(self.linear_target, wait=False)

    def trackbar_left(self, value):
        self.gripper_l_target = int(value)
        self.get_logger().info(f'Moving left gripper to {self.gripper_l_target}')
        self.arm_l.set_gripper_position(self.gripper_l_target, wait=False)

    def trackbar_right(self, value):
        self.gripper_r_target = int(value)
        self.get_logger().info(f'Moving right gripper to {self.gripper_r_target}')
        self.arm_r.set_gripper_position(self.gripper_r_target, wait=False)

    # === Timer callback ===
    def timer_callback(self):
        # --- Linear motor position ---
        code, pos = self.arm_r.get_linear_motor_pos()
        if code == 0:
            msg_lin = Float64()
            pos = float(pos)
            pos_is = (pos * -0.001) + 0.0681
            pos_is = round(pos_is, 3)
            msg_lin.data = pos_is
            self.publisher_linear.publish(msg_lin)
            self.get_logger().info(f'Linear motor position → Isaac: {pos_is}')

        # --- Gripper Left ---
        l = self.gripper_l_target * -0.001 + 0.85
        l = round(l, 3)
        msg_l = Float64()
        msg_l.data = l
        self.publisher_gripper_l.publish(msg_l)

        # --- Gripper Right ---
        r = self.gripper_r_target * -0.001 + 0.85
        r = round(r, 3)
        msg_r = Float64()
        msg_r.data = r
        self.publisher_gripper_r.publish(msg_r)

        # --- Keep UI alive ---
        blank = np.ones((120, 500, 3), dtype=np.uint8) * 255
        cv2.imshow(self.window_name, blank)
        cv2.waitKey(1)

    # === Shutdown ===
    def shutdown(self):
        self.arm_r.set_linear_motor_stop()
        self.arm_r.set_linear_motor_enable(False)
        self.arm_l.set_gripper_enable(False)
        self.arm_r.set_gripper_enable(False)


def main(args=None):
    rclpy.init(args=args)
    node = MultiDevicePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
