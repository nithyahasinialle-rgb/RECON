#!/usr/bin/env python3
"""
Wi-Fi RSSI RF Sensing Simulator Node
Computes Log-Distance Path Loss RSSI values from 3 static Wi-Fi Access Points to the rover's antenna.
RSSI(d) = P0 - 10 * n * log10(d) + noise
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point
from rf_msgs.msg import WiFiRSSI


class WiFiRSSISimulatorNode(Node):

    def __init__(self):
        super().__init__('wifi_rssi_sim_node')

        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('rssi_at_1m_dbm', -40.0)
        self.declare_parameter('path_loss_exponent', 2.5)
        self.declare_parameter('shadowing_std_db', 1.8)

        self.rate = self.get_parameter('update_rate_hz').value
        self.p0 = self.get_parameter('rssi_at_1m_dbm').value
        self.n_exponent = self.get_parameter('path_loss_exponent').value
        self.shadow_std = self.get_parameter('shadowing_std_db').value

        # 3 Static Access Points in 10x10m arena
        self.access_points = [
            {'id': 1, 'ssid': 'Research_AP_01', 'bssid': '00:11:22:33:44:01', 'pose': (0.0, 0.0, 1.5)},
            {'id': 2, 'ssid': 'Research_AP_02', 'bssid': '00:11:22:33:44:02', 'pose': (10.0, 0.0, 1.5)},
            {'id': 3, 'ssid': 'Research_AP_03', 'bssid': '00:11:22:33:44:03', 'pose': (5.0, 10.0, 1.5)},
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.pub_rssi = self.create_publisher(WiFiRSSI, '/wifi/rssi', qos)
        self.timer = self.create_timer(1.0 / self.rate, self.publish_rssi)

        self.get_logger().info('Wi-Fi RSSI Simulator running at %.1f Hz (3 Access Points)' % self.rate)

    def publish_rssi(self):
        try:
            if not self.tf_buffer.can_transform('map', 'wifi_antenna_link', rclpy.time.Time()):
                return

            trans = self.tf_buffer.lookup_transform('map', 'wifi_antenna_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            rz = trans.transform.translation.z

            for ap in self.access_points:
                ax, ay, az = ap['pose']
                dist = math.sqrt((rx - ax)**2 + (ry - ay)**2 + (rz - az)**2)
                dist = max(0.1, dist)

                # Log-Distance Path Loss RSSI equation
                rssi_val = self.p0 - (10.0 * self.n_exponent * math.log10(dist)) + random.gauss(0.0, self.shadow_std)
                rssi_val = max(-95.0, min(-30.0, rssi_val))

                msg = WiFiRSSI()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'wifi_antenna_link'
                msg.bssid = ap['bssid']
                msg.ssid = ap['ssid']
                msg.ap_id = ap['id']
                msg.rssi = float(rssi_val)
                msg.path_loss_exponent = float(self.n_exponent)
                msg.rssi_at_1m = float(self.p0)
                msg.ap_location = Point(x=ax, y=ay, z=az)

                self.pub_rssi.publish(msg)

        except Exception as ex:
            self.get_logger().debug('TF lookup waiting: %s' % str(ex))


def main(args=None):
    rclpy.init(args=args)
    node = WiFiRSSISimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
