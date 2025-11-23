#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from isaac_ros2_messages.srv import SetPrimAttribute

class IsaacBridgeLM(Node):
    def __init__(self):
        super().__init__('isaac_bridge_lm')
        self.subscription = self.create_subscription(Float64, '/linear_motor/position', self.listener_callback, 10)
        self.client = self.create_client(SetPrimAttribute, '/set_prim_attribute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_prim_attribute service...')
        self.y = 0.54
        self.z = 1.022
        self.path = '/World/TABLE_ROB/linear_motor'
        self.attribute = 'xformOp:translate'
        self.latest_value = 0.07
        self.timer = self.create_timer(0.11, self.timer_callback)
        self.get_logger().info('IsaacBridgeLM initialized with 0.11s update rate.')

    def listener_callback(self, msg):
        self.latest_value = float(msg.data)

    def timer_callback(self):
        if not self.client.service_is_ready():
            return
        # Format value as string (Isaac expects a string)
        value_str = f"[{self.latest_value:.3f}, {self.y}, {self.z}]"
        request = SetPrimAttribute.Request()
        request.path = self.path
        request.attribute = self.attribute
        request.value = value_str
        self.client.call_async(request)
        self.get_logger().info(f'Updated prim position -> {value_str}')

def main(args=None):
    rclpy.init(args=args)
    node = IsaacBridgeLM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
