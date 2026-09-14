import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from recon_msgs.msg import CSIData
import serial
import smbus2
import re
import math
class SensorHubNode(Node):
    def __init__(self):
        super().__init__('sensor_hub_node')
        
        # ROS 2 Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.csi_pub = self.create_publisher(CSIData, '/wifi/csi', 10)
        
        # Hardware Setup
        self.setup_imu()
        self.setup_serial()
        
        # Timers (Instead of threads)
        self.create_timer(0.01, self.publish_imu)  # 100Hz IMU read
        self.create_timer(0.02, self.read_serial)  # 50Hz Serial read
        
        self.get_logger().info("✅ RECON Sensor Hub Started!")
    def setup_imu(self):
        self.imu_addr = 0x68
        try:
            self.bus = smbus2.SMBus(1)
            self.bus.write_byte_data(self.imu_addr, 0x6B, 0x00) # Wake up
            self.get_logger().info("MPU6500 Initialized via I2C.")
        except Exception as e:
            self.get_logger().error(f"IMU Error: {e}")
            self.bus = None
    def setup_serial(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.01)
            self.regex = re.compile(r"\[(.*?)\]")
            self.get_logger().info("ESP32 Serial Connected.")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            self.ser = None
    def read_raw_data(self, addr):
        try:
            high = self.bus.read_byte_data(self.imu_addr, addr)
            low = self.bus.read_byte_data(self.imu_addr, addr+1)
            val = ((high << 8) | low)
            return val - 65536 if val > 32768 else val
        except:
            return 0
    def publish_imu(self):
        if not self.bus: return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        # Standard ROS 2 format expects m/s^2 for accel, rad/s for gyro
        msg.linear_acceleration.x = self.read_raw_data(0x3B) / 16384.0 * 9.81
        msg.linear_acceleration.y = self.read_raw_data(0x3D) / 16384.0 * 9.81
        msg.linear_acceleration.z = self.read_raw_data(0x3F) / 16384.0 * 9.81
        msg.angular_velocity.x = math.radians(self.read_raw_data(0x43) / 131.0)
        msg.angular_velocity.y = math.radians(self.read_raw_data(0x45) / 131.0)
        msg.angular_velocity.z = math.radians(self.read_raw_data(0x47) / 131.0)
        
        self.imu_pub.publish(msg)
    def read_serial(self):
        if not self.ser: return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                match = self.regex.search(line)
                if match:
                    vals = [int(x) for x in match.group(1).split(',')]
                    if len(vals) >= 128:
                        msg = CSIData()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "esp32_link"
                        msg.raw_csi = vals[:128] # Enforce fixed size
                        self.csi_pub.publish(msg)
        except Exception:
            pass
def main():
    rclpy.init()
    node = SensorHubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
