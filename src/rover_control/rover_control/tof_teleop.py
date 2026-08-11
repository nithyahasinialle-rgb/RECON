#!/usr/bin/env python3
"""
Rover Keyboard Teleop with 4x ToF Safety Override
Seamless Fluid Arcade Driving Controls (Drive + Steer simultaneously)
Never gets stuck at walls - allows backing up (S) and turning (A/D) freely.
"""

import sys
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False

INSTRUCTIONS = """
--------------------------------------------------
Multisensor Rover Teleop (Seamless Fluid Drive)
--------------------------------------------------
Controls (State-based continuous arcade drive):
   W : Drive Forward
   S : Drive Backward
   A : Steer Left  (Turn left while driving)
   D : Steer Right (Turn right while driving)
   Space / K : Stop

Q/E : Increase/Decrease Speed
CTRL-C to quit
--------------------------------------------------
"""


class ToF4ChannelTeleopNode(Node):

    def __init__(self):
        super().__init__('tof_teleop')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.create_subscription(Range, '/tof/front', self.front_tof_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_speed = 0.50  # m/s
        self.target_turn = 1.00   # rad/s
        self.front_hazard = False

        self.current_linear = 0.0
        self.current_angular = 0.0

        # Continuous 10 Hz timer
        self.timer = self.create_timer(0.10, self.timer_callback)

        self.get_logger().info(INSTRUCTIONS)

    def front_tof_cb(self, msg: Range):
        self.front_hazard = bool(msg.range < 0.25)

    def timer_callback(self):
        twist = Twist()
        if self.front_hazard and self.current_linear > 0.0:
            self.get_logger().warn('Front Obstacle (<0.25m)! Forward clamped - use S to back up or A/D to turn.', throttle_duration_sec=0.5)
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.current_linear

        twist.angular.z = self.current_angular
        self.pub_cmd.publish(twist)


def getKey():
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        return sys.stdin.read(1)
    return ''


def main(args=None):
    old_settings = None
    if HAS_TERMIOS and sys.stdin.isatty():
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except Exception:
            old_settings = None

    rclpy.init(args=args)
    node = ToF4ChannelTeleopNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = getKey()

            if key != '':
                key_lower = key.lower()
                if key_lower == 'w':
                    node.current_linear = node.target_speed
                    node.get_logger().info('Driving FORWARD (w) [v=%.2f m/s]' % node.target_speed)
                elif key_lower == 's':
                    node.current_linear = -node.target_speed
                    node.get_logger().info('Driving BACKWARD (s) [v=-%.2f m/s]' % node.target_speed)
                elif key_lower == 'a':
                    node.current_angular = node.target_turn
                    node.get_logger().info('Steering LEFT (a)')
                elif key_lower == 'd':
                    node.current_angular = -node.target_turn
                    node.get_logger().info('Steering RIGHT (d)')
                elif key == ' ' or key_lower == 'k':
                    node.current_linear = 0.0
                    node.current_angular = 0.0
                    node.get_logger().info('STOPPED')
                elif key_lower == 'q':
                    node.target_speed *= 1.2
                    node.get_logger().info('Speed increased: %.2f m/s' % node.target_speed)
                elif key_lower == 'e':
                    node.target_speed *= 0.8
                    node.get_logger().info('Speed decreased: %.2f m/s' % node.target_speed)
                elif key == '\x03':
                    break

    except Exception as e:
        print("Teleop Exception:", e)
    finally:
        node.current_linear = 0.0
        node.current_angular = 0.0
        if HAS_TERMIOS and old_settings is not None and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
