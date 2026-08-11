#!/usr/bin/env python3
"""
4x ST VL53L5CX 360-Degree 4-Side Multi-Zone 3D Time-of-Flight (ToF) Scanner Node
Accumulates permanent, persistent 3D red hit dots mapping out surrounding furniture and walls across ROS 2 topics.
"""

import math
import random
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from rf_msgs.msg import ToFRange


class ToF4ChannelPublisherNode(Node):

    def __init__(self):
        super().__init__('tof_publisher_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.declare_parameter('min_range_m', 0.03)
        self.declare_parameter('max_range_m', 2.00)
        self.declare_parameter('noise_std_m', 0.008)
        self.declare_parameter('safety_threshold_m', 0.25)

        self.min_range = self.get_parameter('min_range_m').value
        self.max_range = self.get_parameter('max_range_m').value
        self.noise_std = self.get_parameter('noise_std_m').value
        self.safety_dist = self.get_parameter('safety_threshold_m').value

        self.robot_x = 1.5
        self.robot_y = 1.5
        self.robot_yaw = 0.0

        # Subscriptions
        self.create_subscription(PoseStamped, '/rf_slam/pose', self.pose_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Standard Publishers
        self.sensor_configs = [
            ('front', '/tof/front', '/tof/front/custom', 'tof_front_link', 1, 0.0),
            ('left',  '/tof/left',  '/tof/left/custom',  'tof_left_link',  2, math.pi / 2.0),
            ('right', '/tof/right', '/tof/right/custom', 'tof_right_link', 3, -math.pi / 2.0),
            ('rear',  '/tof/rear',  '/tof/rear/custom',  'tof_rear_link',  4, math.pi),
        ]

        self.std_publishers = {}
        self.custom_publishers = {}

        for name, std_top, cust_top, frame_id, id_num, offset in self.sensor_configs:
            self.std_publishers[name] = self.create_publisher(Range, std_top, 10)
            self.custom_publishers[name] = self.create_publisher(ToFRange, cust_top, 10)

        # Permanent MarkerArray & Single Marker Publishers
        self.pub_cloud = self.create_publisher(PointCloud2, '/tof/pointcloud', 10)
        self.pub_rays = self.create_publisher(MarkerArray, '/tof/ray_beams', 10)
        self.pub_hits = self.create_publisher(MarkerArray, '/tof/red_hit_dots', 10)
        self.pub_single_hits = self.create_publisher(Marker, '/tof/single_red_hit_dots', 10)

        self.accumulated_cloud_points = []

        # 20 Hz timer loop
        self.timer = self.create_timer(0.05, self.publish_tof_rays)

        self.get_logger().info('ToF 3D Permanent Map Scanner Active')

    def pose_cb(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def ray_cast_distance(self, ray_heading, elevation_angle=0.0):
        rx, ry = self.robot_x, self.robot_y
        dx = math.cos(ray_heading) * math.cos(elevation_angle)
        dy = math.sin(ray_heading) * math.cos(elevation_angle)

        min_d = self.max_range

        # 1. Outer Room Walls (x=0, x=10, y=0, y=10)
        if abs(dx) > 1e-4:
            d_east = (10.0 - rx) / dx
            if d_east > 0 and d_east < min_d: min_d = d_east
            d_west = -rx / dx
            if d_west > 0 and d_west < min_d: min_d = d_west

        if abs(dy) > 1e-4:
            d_north = (10.0 - ry) / dy
            if d_north > 0 and d_north < min_d: min_d = d_north
            d_south = -ry / dy
            if d_south > 0 and d_south < min_d: min_d = d_south

        # 2. Indoor Furniture Objects (Conference Tables, Cabinets, Bookshelves)
        furniture_boxes = [
            (3.5, 3.5, 1.6, 0.9),  # Conference Table 1
            (6.5, 6.5, 1.6, 0.9),  # Conference Table 2
            (1.2, 7.5, 0.6, 1.4),  # Storage Cabinet
            (8.8, 2.5, 0.5, 1.2),  # Bookshelf Unit
        ]

        for cx, cy, sx, sy in furniture_boxes:
            min_x, max_x = cx - sx/2.0, cx + sx/2.0
            min_y, max_y = cy - sy/2.0, cy + sy/2.0

            if abs(dx) > 1e-4:
                tx1 = (min_x - rx) / dx
                tx2 = (max_x - rx) / dx
                for t in [tx1, tx2]:
                    if t > 0 and t < min_d:
                        py_hit = ry + t * dy
                        if min_y <= py_hit <= max_y:
                            min_d = t

            if abs(dy) > 1e-4:
                ty1 = (min_y - ry) / dy
                ty2 = (max_y - ry) / dy
                for t in [ty1, ty2]:
                    if t > 0 and t < min_d:
                        px_hit = rx + t * dx
                        if min_x <= px_hit <= max_x:
                            min_d = t

        return min_d

    def publish_tof_rays(self):
        now_time = self.get_clock().now().to_msg()
        ray_markers = MarkerArray()

        for name, std_top, cust_top, frame_id, id_num, angle_offset in self.sensor_configs:
            ray_angle = self.robot_yaw + angle_offset
            raw_dist = self.ray_cast_distance(ray_angle)

            dist = raw_dist if raw_dist >= self.max_range else max(self.min_range, min(self.max_range, raw_dist + random.gauss(0.0, self.noise_std)))
            is_valid = bool(dist < self.max_range)

            range_msg = Range()
            range_msg.header.stamp = now_time
            range_msg.header.frame_id = frame_id
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.436
            range_msg.min_range = self.min_range
            range_msg.max_range = self.max_range
            range_msg.range = float(dist)
            self.std_publishers[name].publish(range_msg)

            tof_msg = ToFRange()
            tof_msg.header.stamp = now_time
            tof_msg.header.frame_id = frame_id
            tof_msg.sensor_id = id_num
            tof_msg.sensor_label = name
            tof_msg.distance = float(dist)
            tof_msg.min_range = self.min_range
            tof_msg.max_range = self.max_range
            tof_msg.field_of_view = 0.436
            tof_msg.range_valid = is_valid
            tof_msg.hazard_warning = bool(dist < self.safety_dist and is_valid)
            self.custom_publishers[name].publish(tof_msg)

            sensor_origin_x = self.robot_x + 0.12 * math.cos(ray_angle)
            sensor_origin_y = self.robot_y + 0.12 * math.sin(ray_angle)
            sensor_origin_z = 0.10

            fov_azimuth = 0.45
            fov_elevation = 0.45
            grid_size = 6

            ray_line_marker = Marker()
            ray_line_marker.header.stamp = now_time
            ray_line_marker.header.frame_id = "odom"
            ray_line_marker.ns = "tof_ray_beams"
            ray_line_marker.id = id_num
            ray_line_marker.type = Marker.LINE_LIST
            ray_line_marker.action = Marker.ADD
            ray_line_marker.scale.x = 0.008
            ray_line_marker.color.r = 0.0f
            ray_line_marker.color.g = 0.85f
            ray_line_marker.color.b = 1.0f
            ray_line_marker.color.a = 0.75f

            for row in range(grid_size):
                for col in range(grid_size):
                    az = (col / (grid_size - 1) - 0.5) * fov_azimuth
                    el = (row / (grid_size - 1) - 0.5) * fov_elevation
                    zone_heading = ray_angle + az
                    d_zone = self.ray_cast_distance(zone_heading, el)

                    if d_zone < self.max_range:
                        pt_x = self.robot_x + d_zone * math.cos(zone_heading) * math.cos(el)
                        pt_y = self.robot_y + d_zone * math.sin(zone_heading) * math.cos(el)
                        pt_z = 0.10 + d_zone * math.sin(el)

                        self.add_cloud_point(pt_x, pt_y, pt_z)

                        p_start = Point(x=sensor_origin_x, y=sensor_origin_y, z=sensor_origin_z)
                        p_end = Point(x=pt_x, y=pt_y, z=pt_z)
                        ray_line_marker.points.append(p_start)
                        ray_line_marker.points.append(p_end)

            if ray_line_marker.points:
                ray_markers.markers.push_back(ray_line_marker)

        if ray_markers.markers:
            self.pub_rays.publish(ray_markers)

        if self.accumulated_cloud_points:
            self.publish_pointcloud(self.accumulated_cloud_points, now_time)
            self.publish_permanent_red_hit_dots(self.accumulated_cloud_points, now_time)

    def add_cloud_point(self, x, y, z):
        # Deduplicate points within 3cm
        for px, py, pz in self.accumulated_cloud_points[-400:]:
            if math.hypot(px - x, py - y) < 0.03 and abs(pz - z) < 0.03:
                return
        self.accumulated_cloud_points.append((x, y, z))
        if len(self.accumulated_cloud_points) > 20000:
            self.accumulated_cloud_points.pop(0)

    def publish_pointcloud(self, points, stamp):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width

        buffer = bytearray()
        for pt in points:
            buffer.extend(struct.pack('fff', pt[0], pt[1], pt[2]))

        msg.data = bytes(buffer)
        self.pub_cloud.publish(msg)

    def publish_permanent_red_hit_dots(self, points, stamp):
        hit_markers = MarkerArray()
        # Publish ALL accumulated points permanently (up to 1,000 for MarkerArray)
        for idx, (px, py, pz) in enumerate(points[-1000:]):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "odom"
            m.ns = "tof_red_hit_dots"
            m.id = idx + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = pz
            m.scale.x = 0.08
            m.scale.y = 0.08
            m.scale.z = 0.08
            m.color.r = 1.0f
            m.color.g = 0.1f
            m.color.b = 0.2f
            m.color.a = 0.85f
            hit_markers.markers.push_back(m)

        if hit_markers.markers:
            self.pub_hits.publish(hit_markers)

        # Publish single Marker SPHERE_LIST containing ALL accumulated points permanently
        single_m = Marker()
        single_m.header.stamp = stamp
        single_m.header.frame_id = "odom"
        single_m.ns = "single_red_hit_dots"
        single_m.id = 888
        single_m.type = Marker.SPHERE_LIST
        single_m.action = Marker.ADD
        single_m.scale.x = 0.08
        single_m.scale.y = 0.08
        single_m.scale.z = 0.08
        single_m.color.r = 1.0f
        single_m.color.g = 0.1f
        single_m.color.b = 0.2f
        single_m.color.a = 0.85f

        for px, py, pz in points[-5000:]:
            single_m.points.append(Point(x=px, y=py, z=pz))

        self.pub_single_hits.publish(single_m)


def main(args=None):
    rclpy.init(args=args)
    node = ToF4ChannelPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
