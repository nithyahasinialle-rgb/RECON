#!/usr/bin/env python3
"""
===============================================================================
IEEE Conference / Competition Benchmark Suite: Multi-Sensor RF SLAM & ToF Mapping
===============================================================================
Autonomous Indoor Office/Lab Navigation & Performance Evaluation Script
Navigates the rover through a 14-stage IEEE evaluation trail visiting:
 1. Conference Table 1 Area (Oak Wood Setup @ 3.5, 3.5)
 2. Storage Cabinet Inspection Zone (@ 1.2, 7.5)
 3. Conference Table 2 Area (Walnut Wood Setup @ 6.5, 6.5)
 4. Bookshelf & Equipment Station (@ 8.8, 2.5)
 5. Perimeter Wall Alignment Sweep
 6. Precision Return to Origin (1.5, 1.5)

Outputs: IEEE_Indoor_Lab_Benchmark_Results.csv
===============================================================================
"""

import sys
import time
import math
import csv
import psutil
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

# IEEE Indoor Office Target Feature Waypoints
IEEE_STATIONS = [
    (1.5, 1.5, "Origin_Home_Base"),
    (2.8, 2.8, "Approaching_Conference_Table_1"),
    (3.5, 2.3, "Table_1_South_Inspection"),
    (4.5, 3.5, "Table_1_East_Inspection"),
    (3.5, 4.7, "Table_1_North_Inspection"),
    (1.5, 6.0, "Transit_Storage_Cabinet_Zone"),
    (1.5, 7.5, "Storage_Cabinet_Inspection"),
    (4.5, 7.5, "Transit_Table_2_Zone"),
    (6.5, 5.5, "Table_2_South_Inspection"),
    (7.5, 6.5, "Table_2_East_Inspection"),
    (8.5, 4.0, "Transit_Bookshelf_Station"),
    (8.5, 2.5, "Bookshelf_Equipment_Inspection"),
    (8.5, 1.5, "Perimeter_South_East_Wall"),
    (1.5, 1.5, "IEEE_Evaluation_Complete_Home"),
]


class IEEEMultiSensorBenchmarkNode(Node):

    def __init__(self, duration_sec=120, csv_filename="IEEE_Indoor_Lab_Benchmark_Results.csv"):
        super().__init__('ieee_coverage_benchmark')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/rf_slam/pose', self.pose_cb, 10)
        self.sub_path = self.create_subscription(Path, '/rf_slam/trajectory', self.path_cb, 10)
        
        # Subscriptions to BOTH ToF live hit dots and SLAM obstacle map
        self.sub_tof_obs = self.create_subscription(MarkerArray, '/tof/red_hit_dots', self.tof_obs_cb, 10)
        self.sub_slam_obs = self.create_subscription(MarkerArray, '/rf_slam/obstacles', self.slam_obs_cb, 10)

        self.robot_x = 1.5
        self.robot_y = 1.5
        self.robot_yaw = 0.0

        self.station_idx = 0
        self.total_stations = len(IEEE_STATIONS)
        self.trajectory_poses = 0
        self.tof_mapped_points = 0
        self.slam_mapped_points = 0
        self.total_distance_traveled = 0.0
        self.prev_x = 1.5
        self.prev_y = 1.5

        self.duration_sec = duration_sec
        self.csv_filename = csv_filename
        self.start_time = time.time()

        # IEEE CSV Data Logging
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "Timestamp_Sec", "Station_ID", "Target_Feature_Name",
            "Target_X_m", "Target_Y_m", "Robot_X_m", "Robot_Y_m", "Heading_Deg",
            "Distance_Traveled_m", "Mapped_Obstacle_Points", "Trajectory_Poses",
            "Jetson_CPU_Load_Pct", "Jetson_RAM_Used_MB", "System_RAM_Pct"
        ])

        # 10 Hz Control Timer & 1 Hz Benchmark Logger
        self.control_timer = self.create_timer(0.10, self.control_loop)
        self.log_timer = self.create_timer(1.00, self.log_benchmark)

        self.get_logger().info("=" * 75)
        self.get_logger().info("IEEE Event Multi-Sensor SLAM Evaluation Suite Active (%ds Run)" % self.duration_sec)
        self.get_logger().info("Target CSV: %s" % self.csv_filename)
        self.get_logger().info("=" * 75)

    def pose_cb(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        d_step = math.hypot(self.robot_x - self.prev_x, self.robot_y - self.prev_y)
        if d_step > 0.005 and d_step < 0.5:
            self.total_distance_traveled += d_step
            self.prev_x = self.robot_x
            self.prev_y = self.robot_y

    def path_cb(self, msg: Path):
        self.trajectory_poses = len(msg.poses)

    def tof_obs_cb(self, msg: MarkerArray):
        self.tof_mapped_points = len(msg.markers)

    def slam_obs_cb(self, msg: MarkerArray):
        self.slam_mapped_points = len(msg.markers)

    def control_loop(self):
        if self.station_idx >= self.total_stations:
            self.stop_rover()
            return

        tx, ty, name = IEEE_STATIONS[self.station_idx]
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        dist = math.hypot(dx, dy)

        mapped_count = max(self.tof_mapped_points, self.slam_mapped_points)

        if dist < 0.35:  # Station reached
            self.get_logger().info("[IEEE EVAL] Reached Station %d/%d: %s (Mapped Points: %d, Path Length: %.2fm)" %
                                   (self.station_idx + 1, self.total_stations, name, mapped_count, self.total_distance_traveled))
            self.station_idx += 1
            if self.station_idx >= self.total_stations:
                self.get_logger().info("IEEE Multi-Sensor Indoor Mapping Trial Completed Successfully!")
                self.stop_rover()
                return
            tx, ty, name = IEEE_STATIONS[self.station_idx]
            dx = tx - self.robot_x
            dy = ty - self.robot_y

        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.robot_yaw

        while heading_error > math.pi: heading_error -= 2.0 * math.pi
        while heading_error < -math.pi: heading_error += 2.0 * math.pi

        twist = Twist()
        if abs(heading_error) > 0.35:
            twist.linear.x = 0.08
            twist.angular.z = 1.1 if heading_error > 0 else -1.1
        else:
            twist.linear.x = min(0.50, 0.35 + dist * 0.25)
            twist.angular.z = 0.75 * heading_error

        self.pub_cmd.publish(twist)

    def stop_rover(self):
        twist = Twist()
        self.pub_cmd.publish(twist)

    def log_benchmark(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration_sec:
            self.get_logger().info("IEEE Benchmark Run Completed (%ds). Writing final report..." % self.duration_sec)
            self.stop_rover()
            self.csv_file.close()
            rclpy.shutdown()
            return

        cpu_pct = psutil.cpu_percent()
        ram_mem = psutil.virtual_memory()
        ram_mb = ram_mem.used / (1024.0 * 1024.0)
        ram_pct = ram_mem.percent

        mapped_count = max(self.tof_mapped_points, self.slam_mapped_points)
        tx, ty, name = IEEE_STATIONS[min(self.station_idx, self.total_stations - 1)]

        self.csv_writer.writerow([
            "%.2f" % elapsed, self.station_idx + 1, name,
            "%.2f" % tx, "%.2f" % ty, "%.2f" % self.robot_x, "%.2f" % self.robot_y,
            "%.1f" % math.degrees(self.robot_yaw), "%.2f" % self.total_distance_traveled,
            mapped_count, self.trajectory_poses,
            "%.1f" % cpu_pct, "%.1f" % ram_mb, "%.1f" % ram_pct
        ])
        self.csv_file.flush()

        self.get_logger().info(
            "[IEEE t=%3.0fs] Station %2d/%2d: %-32s | Pos:(%4.2f,%4.2f) | Obstacles Mapped: %3d | Path: %5.2fm | CPU: %4.1f%% | RAM: %5.1fMB" %
            (elapsed, self.station_idx + 1, self.total_stations, name, self.robot_x, self.robot_y,
             mapped_count, self.total_distance_traveled, cpu_pct, ram_mb)
        )


def main(args=None):
    duration = 120
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            pass

    rclpy.init(args=args)
    node = IEEEMultiSensorBenchmarkNode(duration_sec=duration)
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
