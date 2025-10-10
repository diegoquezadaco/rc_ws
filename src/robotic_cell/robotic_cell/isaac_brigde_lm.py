# Isaac Sim script: linear_motor_to_prim.py
# Paste into Isaac Sim Script Editor and Run.
# Assumptions:
#  - ROS2 is available inside Isaac Sim's Python (rclpy import succeeds)
#  - /linear_motor/position publishes std_msgs/Float64 with motor position (units you know)
#
# Tweak the PARAMETERS section to match your motor's numeric range and mapping preference.

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from pxr import Usd, UsdGeom, Sdf
import omni.usd

# ---------------- PARAMETERS (tweak these) ----------------
PRIM_PATH = "/World/TABLE_ROB/AL0700_Zero_position_/tn__AL0700Zeroposition_yIg4f0/tn__1solid1_oA2qc0b2z0e0k0t5Z0l4V2sK"

# The prim Z value limits you gave in the UI (these are stage units seen in the property window)
PRIM_Z_MIN = -745.0
PRIM_Z_MAX = 70.0


# Poll frequency for applying updates (seconds)
POLL_INTERVAL = 0.05

# If your motor reports millimeters and your stage expects meters, you can alter mapping style below.
# We use a normalized mapping: motor [MOTOR_MIN..MOTOR_MAX] --> prim Z [PRIM_Z_MIN..PRIM_Z_MAX]
# To change to "offset + scale" mapping instead, see the "ALTERNATIVE MAPPING" notes in the guide.
# --------------------------------------------------------

def clamp(v, lo, hi):
    return max(lo, min(v, hi))

class Ros2Listener(Node):
    def __init__(self):
        super().__init__('isaac_linear_listener')
        self._sub = self.create_subscription(Float64, '/linear_motor/position', self._cb, 10)
        self._latest = 0.0
        self._has_msg = False

    def _cb(self, msg):
        # msg.data is whatever your SDK publishes (e.g. mm)
        self._latest = float(msg.data)
        self._has_msg = True

    def latest(self):
        return self._latest, self._has_msg

def start_ros2_listener():
    # initialize rclpy and spin in a background thread
    rclpy.init()
    node = Ros2Listener()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception as e:
            print("ROS2 executor stopped:", e)

    t = threading.Thread(target=spin, daemon=True)
    t.start()
    return node, executor, t


# ----------------- Start everything -----------------
ros_node, ros_executor, ros_thread = start_ros2_listener()
print("[linear_motor_to_prim] ROS2 listener started (subscribing /linear_motor/position)")

# get stage and prim
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath(Sdf.Path(PRIM_PATH))
if not prim or not prim.IsValid():
    raise RuntimeError(f"Prim not found at {PRIM_PATH} - please set PRIM_PATH correctly")

xform_api = UsdGeom.XformCommonAPI(prim)

running = True

def update_loop():
    last_applied = None
    while running:
        motor_value, has = ros_node.latest()
        if has:
            z = motor_value
            # Set translation (X, Y, Z). Keep X and Y as-is (we only update Z).
            # Read current translate if present, otherwise use zero for X,Y.
            try:
                current = xform_api.GetTranslateAttr().Get()
                if current is None:
                    current = (0.0, 0.0, 0.0)
            except Exception:
                # if attribute missing, default
                current = (0.0, 0.0, 0.0)
            new_translate = (current[0], current[1], float(z))
            # Apply transform
            xform_api.SetTranslate(new_translate)
            # Optional: print debug sometimes
            if last_applied is None or abs(last_applied - z) > 0.001:
                print(f"[linear_motor_to_prim] motor={motor_value:.3f} -> prim Z={z:.3f}")
                last_applied = z
        time.sleep(POLL_INTERVAL)

update_thread = threading.Thread(target=update_loop, daemon=True)
update_thread.start()

print("[linear_motor_to_prim] update loop running. To stop: call stop_linear_bridge()")

# Convenient cleanup function you can call from the Script Editor to stop the background threads cleanly:
def stop_linear_bridge():
    global running, ros_executor
    print("[linear_motor_to_prim] stopping...")
    running = False
    try:
        if ros_executor:
            ros_executor.shutdown()
    except Exception as e:
        print("Error shutting down ros executor:", e)
    try:
        rclpy.shutdown()
    except Exception as e:
        print("Error shutting down rclpy:", e)
    print("[linear_motor_to_prim] stopped.")