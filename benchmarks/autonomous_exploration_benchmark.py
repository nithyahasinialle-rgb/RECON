#!/usr/bin/env python3
"""
Autonomous Multi-Sensor Exploration & Smart Obstacle Escape Benchmark Script
Explores the 10x10m arena, automatically detecting pillars/walls, backing up smoothly,
steering toward open space, and building a complete SLAM map without getting stuck.
"""

import sys
import time
import math
import random
import csv
import psutil
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

STATE_FORWARD = 0
STATE_BACKING_UP = 1
STATE_TURNING = 2


class AutonomousExplorationBenchmarkNode(Node):

    def __init__(self, duration_sec=120, csv_filename="autonomous_exploration_benchmark.csv"):
        super().__init__('autonomous_exploration_benchmark')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/rf_slam/pose', self.pose_cb, 10)
        self.sub_path = self.create_subscription(Path, '/rf_slam/trajectory', self.path_cb, 10)
        self.sub_obs = self.create_subscription(MarkerArray, '/rf_slam/obstacles', self.obs_cb, 10)

        # 4x ToF sensor subscriptions
        self.create_subscription(Range, '/tof/front', self.front_cb, 10)
        self.create_subscription(Range, '/tof/left',  self.left_cb,  10)
        self.create_subscription(Range, '/tof/right', self.right_cb, 10)

        self.front_dist = 2.0
        self.left_dist = 2.0
        self.right_dist = 2.0

        self.robot_x = 1.5
        self.robot_y = 1.5
        self.robot_yaw = 0.0

        self.mapped_obstacles = 0
        self.trajectory_points = 0

        self.state = STATE_FORWARD
        self.state_start_time = time.time()
        self.turn_direction = 1.0  # +1 for left, -1 for right

        self.duration_sec = duration_sec
        self.csv_filename = csv_filename
        self.start_time = time.time()

        # CSV Logging Setup
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "Timestamp_Sec", "State", "Robot_X", "Robot_Y", "Robot_Yaw_Deg",
            "Front_ToF_m", "Left_ToF_m", "Right_ToF_m",
            "Mapped_Obstacles", "Trajectory_Points", "CPU_Usage_Pct", "RAM_Usage_MB"
        ])

        # Control Loop @ 10 Hz
        self.control_timer = self.create_timer(0.10, self.control_loop)
        # Benchmark Logger @ 1 Hz
        self.log_timer = self.create_timer(1.00, self.log_benchmark)

        self.get_logger().info("Autonomous Smart Exploration & Obstacle Escape Benchmark Active (%ds)" % self.duration_sec)

    def pose_cb(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def path_cb(self, msg: Path):
        self.trajectory_points = len(msg.poses)

    def obs_cb(self, msg: MarkerArray):
        self.mapped_obstacles = len(msg.markers)

    def front_cb(self, msg: Range):
        self.front_dist = msg.range

    def left_cb(self, msg: Range):
        self.left_dist = msg.range

    def right_cb(self, msg: Range):
        self.right_dist = msg.range

    def control_loop(self):
        now = time.time()
        dt_state = now - self.state_start_time

        twist = Twist()

        if self.state == STATE_FORWARD:
            if self.front_dist < 0.40:  # Obstacle or pillar detected ahead!
                self.get_logger().info("Obstacle/Pillar Detected Ahead (%.2fm)! Reversing to explore elsewhere..." % self.front_dist)
                self.state = STATE_BACKING_UP
                self.state_start_time = now
                # Turn away from closer obstacle
                self.turn_direction = 1.0 if self.right_dist < self.left_dist else -1.0
            else:
                twist.linear.x = 0.50
                # Slight wander curve to cover full arena space
                twist.angular.z = 0.15 * math.sin(now * 0.5)

        elif self.state == STATE_BACKING_UP:
            if dt_state < 1.2:  # Back up for 1.2 seconds
                twist.linear.x = -0.40
                twist.angular.z = -0.30 * self.turn_direction
            else:
                self.state = STATE_TURNING
                self.state_start_time = now

        elif self.state == STATE_TURNING:
            if dt_state < 1.0:  # Spin 90 degrees to open space
                twist.linear.x = 0.05
                twist.angular.z = 1.4 * self.turn_direction
            else:
                self.state = STATE_FORWARD
                self.state_start_time = now

        self.pub_cmd.publish(twist)

    def stop_rover(self):
        twist = Twist()
        self.pub_cmd.publish(twist)

    def log_benchmark(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration_sec:
            self.get_logger().info("Exploration benchmark complete (%ds). Saving results..." % self.duration_sec)
            self.stop_rover()
            self.csv_file.close()
            rclpy.shutdown()
            return

        cpu_pct = psutil.cpu_percent()
        ram_mem = psutil.virtual_memory()
        ram_mb = ram_mem.used / (1024.0 * 1024.0)

        state_names = {STATE_FORWARD: "FORWARD", STATE_BACKING_UP: "BACKING_UP", STATE_TURNING: "TURNING"}

        self.csv_writer.writerow([
            "%.2f" % elapsed, state_names[self.state],
            "%.2f" % self.robot_x, "%.2f" % self.robot_y, "%.1f" % math.degrees(self.robot_yaw),
            "%.2f" % self.front_dist, "%.2f" % self.left_dist, "%.2f" % self.right_dist,
            self.mapped_obstacles, self.trajectory_points, "%.1f" % cpu_pct, "%.1f" % ram_mb
        ])
        self.csv_file.flush()

        self.get_logger().info(
            "[t=%3.0fs] Mode: %-10s | Pos:(%.2f,%.2f) | Obstacles Mapped: %d | CPU: %.1f%% | RAM: %.1fMB" %
            (elapsed, state_names[self.state], self.robot_x, self.robot_y, self.mapped_obstacles, cpu_pct, ram_mb)
        )


def main(args=None):
    duration = 120
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            pass

    rclpy.init(args=args)
    node = AutonomousExplorationBenchmarkNode(duration_sec=duration)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.stop_rover()
        node.csv_file.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
