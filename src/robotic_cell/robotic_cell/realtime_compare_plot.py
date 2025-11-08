#!/usr/bin/env python3
"""
Real-time comparison of real vs simulated xArm6 joint positions.

Subscribes to:
 - /R_xarm6_traj_controller/state   (control_msgs/JointTrajectoryControllerState)
 - xarm6_sim/joint_states           (sensor_msgs/JointState)

Displays:
 - One live line plot (OpenCV window) with two lines:
     * mean(real joints 1..6)
     * mean(sim joints 1..6)
 - Running RMSE (6-DOF) printed and rendered on the plot.
 - Stops after exactly 45 seconds and prints final RMSE.

Requirements:
 - ROS 2 (rclpy)
 - control_msgs, sensor_msgs
 - numpy, matplotlib, opencv-python

Usage:
    ros2 run <your_pkg> realtime_compare_plot
    OR
    python3 realtime_compare_plot.py
"""

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import numpy as np
import threading
import time
import matplotlib
matplotlib.use("Agg")  # render to image buffer
import matplotlib.pyplot as plt
import cv2
from collections import deque

# Parameters
SAMPLE_HZ = 20
DURATION_SEC = 55.0
WINDOW_WIDTH_PX = 900
WINDOW_HEIGHT_PX = 500

import csv
from datetime import datetime
import os

# Create output directory and CSV file
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir = os.path.join(os.path.expanduser("."), "robotic_cell_logs")
os.makedirs(output_dir, exist_ok=True)
csv_path = os.path.join(output_dir, f"realtime_log_{timestamp}.csv")

# Initialize CSV file with headers
with open(csv_path, mode="w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
    "Timestamp",
    "Real_J1", "Real_J2", "Real_J3", "Real_J4", "Real_J5", "Real_J6",
    "Sim_J1", "Sim_J2", "Sim_J3", "Sim_J4", "Sim_J5", "Sim_J6",
    "RMSE_per_joint", "Avg_RMSE", "TimeDelay_ms"
])



def safe_mean(x):
    return float(np.mean(x)) if len(x) > 0 else 0.0

class RealtimeCompareNode(Node):
    def __init__(self):
        super().__init__('realtime_compare_node')

        # Latest state containers (thread-safe via lock)
        self.lock = threading.Lock()
        # For real arm: we'll expect joint names like R_joint1 ... R_joint6
        self.latest_real = {'names': [], 'positions': None, 'stamp': None}
        # For sim: joint1..joint6
        self.latest_sim = {'names': [], 'positions': None, 'stamp': None}

        # Subscribers
        self.create_subscription(
            JointTrajectoryControllerState,
            '/L_uf850_traj_controller/state',
            self.real_callback,
            10
        )
        self.create_subscription(
            JointState,
            'uf850_sim/joint_states',
            self.sim_callback,
            10
        )

        # Data buffers for plotting
        self.times = []
        self.real_means = []
        self.sim_means = []
        self.rmse_list = []

        # Start sampling thread
        self.running = True
        self.start_time = None
        self.sampler_thread = threading.Thread(target=self.sampler_loop, daemon=True)
        self.sampler_thread.start()

    def real_callback(self, msg: JointTrajectoryControllerState):
        # msg.actual.positions and msg.actual is a trajectory-like structure
        # Names may be in msg.joint_names
        with self.lock:
            names = list(msg.joint_names) if hasattr(msg, 'joint_names') else []
            # get actual.positions if present
            actual = []
            try:
                actual = list(msg.actual.positions)
            except Exception:
                actual = []
            self.latest_real['names'] = names
            self.latest_real['positions'] = np.array(actual, dtype=float) if len(actual) > 0 else None
            self.latest_real['stamp'] = self.get_clock().now()

    def sim_callback(self, msg: JointState):
        with self.lock:
            names = list(msg.name)
            positions = list(msg.position)
            self.latest_sim['names'] = names
            self.latest_sim['positions'] = np.array(positions, dtype=float) if len(positions) > 0 else None
            self.latest_sim['stamp'] = self.get_clock().now()

    def extract_first6(self, names, positions, real=False):
        """
        Map names -> indices for joints 1..6.
        For real, names expected like 'R_joint1'..'R_joint6' or similar.
        For sim, 'joint1'..'joint6'.
        Returns numpy array of length 6 or None if not available.
        """
        if positions is None or names is None or len(names) == 0:
            return None
        arr = np.array(positions, dtype=float)
        # normalize names (strip prefixes)
        target_simple = ['joint1','joint2','joint3','joint4','joint5','joint6']
        # search mapping
        mapping = {}
        for i, n in enumerate(names):
            ln = n.lower()
            # remove common prefixes 'r_' or 'r' at begin
            ln_stripped = ln
            if ln.startswith('r_'):
                ln_stripped = ln[2:]
            elif ln.startswith('l'):
                # maybe 'R_joint1' -> 'r_joint1' handled above, but keep fallback
                ln_stripped = ln[1:] if len(ln) > 1 and ln[1] == '_' else ln
            # also replace '-' or '.' with '_'
            ln_stripped = ln_stripped.replace('-', '_').replace('.', '_')
            mapping[ln_stripped] = i
            mapping[ln] = i

        selected = []
        for t in target_simple:
            if t in mapping:
                idx = mapping[t]
                if idx < arr.shape[0]:
                    selected.append(arr[idx])
                else:
                    selected.append(np.nan)
            else:
                # try patterns like 'r_joint1' or 'rjoint1' for real
                candidates = []
                if real:
                    candidates = [f"l_{t}", f"l{t}"]
                else:
                    candidates = [t]
                found = False
                for c in candidates:
                    if c in mapping:
                        idx = mapping[c]
                        selected.append(arr[idx])
                        found = True
                        break
                if not found:
                    # attempt to find any name containing the joint number
                    num = ''.join(filter(str.isdigit, t))
                    found_idx = None
                    for k, idx in mapping.items():
                        if num and num in k:
                            found_idx = idx
                            break
                    if found_idx is not None:
                        selected.append(arr[found_idx])
                    else:
                        selected.append(np.nan)
        selected = np.array(selected, dtype=float)
        # If all NaN, return None
        if np.all(np.isnan(selected)):
            return None
        # replace NaN by previous valid or 0
        selected = np.nan_to_num(selected, nan=0.0)
        return selected

    def sampler_loop(self):
        rate = 1.0 / SAMPLE_HZ
        self.start_time = time.time()
        end_time = self.start_time + DURATION_SEC
        while self.running and time.time() < end_time and rclpy.ok():
            t = time.time() - self.start_time
            with self.lock:
                real_vals = self.extract_first6(self.latest_real.get('names', []),
                                                self.latest_real.get('positions', None),
                                                real=True)
                sim_vals = self.extract_first6(self.latest_sim.get('names', []),
                                               self.latest_sim.get('positions', None),
                                               real=False)
            # If either None, use zeros or skip? We'll use zeros so graph keeps flowing.
            if real_vals is None:
                real_vals = np.zeros(6)
            if sim_vals is None:
                sim_vals = np.zeros(6)

            # compute mean across 6 joints
            real_mean = float(np.mean(real_vals))
            sim_mean = float(np.mean(sim_vals))
            diff = real_vals - sim_vals
            rmse = float(np.sqrt(np.mean(diff ** 2)))

            # Compute RMSE per joint
            rmse_per_joint = np.sqrt((real_vals - sim_vals) ** 2)
            avg_rmse = float(np.mean(rmse_per_joint))

            # Estimate time delay (ms)
            real_stamp = self.latest_real.get('stamp')
            sim_stamp = self.latest_sim.get('stamp')
            if real_stamp and sim_stamp:
                delay_ms = (real_stamp.nanoseconds - sim_stamp.nanoseconds) / 1e6
            else:
                delay_ms = 0.0

            # Log data to CSV
            with open(csv_path, mode="a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    *real_vals.tolist(),
                    *sim_vals.tolist(),
                    *rmse_per_joint.tolist(),
                    avg_rmse,
                    delay_ms
                ])



            # append
            self.times.append(t)
            self.real_means.append(real_mean)
            self.sim_means.append(sim_mean)
            self.rmse_list.append(rmse)

            # update window
            self.render_and_show(t, real_mean, sim_mean, rmse)

            time.sleep(rate)

        # after loop, compute final RMSE and stop
        final_rmse = np.mean(self.rmse_list) if len(self.rmse_list) > 0 else float('nan')
        self.get_logger().info(f"Finished sampling. Final mean RMSE over {len(self.rmse_list)} samples: {final_rmse:.6f}")
        print(f"FINAL MEAN RMSE: {final_rmse:.6f}")
        self.running = False
        # give a bit to show final frame
        time.sleep(1.0)
        cv2.destroyAllWindows()
        # Save final chart and summary
        final_chart_path = os.path.join(output_dir, f"final_plot_{timestamp}.png")
        fig, ax = plt.subplots(figsize=(WINDOW_WIDTH_PX/100, WINDOW_HEIGHT_PX/100), dpi=100)
        ax.plot(self.times, self.real_means, label='Real mean(1..6)')
        ax.plot(self.times, self.sim_means, label='Sim mean(1..6)')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('mean joint position (rad)')
        ax.set_title('Final Summary: Real vs Sim mean joint positions')
        ax.legend(loc='upper right')
        ax.grid(True)
        plt.tight_layout()
        plt.savefig(final_chart_path, dpi=300)
        plt.close(fig)
        print(f"[INFO] Final chart saved to: {final_chart_path}")
        print(f"[INFO] CSV log saved to: {csv_path}")
        # shutdown ROS
        rclpy.shutdown()

    def render_and_show(self, t, real_mean, sim_mean, rmse):
        # create matplotlib figure and draw the two lines
        fig, ax = plt.subplots(figsize=(WINDOW_WIDTH_PX/100, WINDOW_HEIGHT_PX/100), dpi=100)
        ax.plot(self.times, self.real_means, label='Real mean(1..6)')
        ax.plot(self.times, self.sim_means, label='Sim mean(1..6)')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('mean joint position (rad)')
        ax.set_title('Real vs Sim — mean joint position (6 joints)')
        ax.legend(loc='upper right')
        # annotation: latest RMSE and running average RMSE
        running_avg_rmse = np.mean(self.rmse_list) if len(self.rmse_list) > 0 else 0.0
        textstr = f'Latest RMSE: {rmse:.6f}\\nMean RMSE: {running_avg_rmse:.6f}\\nElapsed: {t:.1f}s'
        ax.text(0.02, 0.95, textstr, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', bbox=dict(boxstyle='round', alpha=0.2))
        ax.grid(True)

        # render figure to image
        fig.tight_layout()
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())
        w, h = fig.canvas.get_width_height()
        img = img.reshape((h, w, 4))
        plt.close(fig)

        # convert RGBA to BGR for OpenCV
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        # resize to desired window
        img_bgr = cv2.resize(img_bgr, (WINDOW_WIDTH_PX, WINDOW_HEIGHT_PX))
        cv2.imshow('Real vs Sim (mean joints) - Press q to quit', img_bgr)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("User requested quit (q).")
            self.running = False

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeCompareNode()
    try:
        # spin in separate thread to process subscriptions
        executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        executor_thread.start()

        # wait for sampler thread to finish
        node.sampler_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.running = False
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
