# Custom ROS 2 Message for UWB RF Ranging Data
# Designed for Jetson Orin Nano embedded RF-SLAM pipeline

std_msgs/Header header          # Timestamp and frame_id (e.g. uwb_tag_link)

uint32 anchor_id                # Unique ID of the transmitting RF Anchor
string anchor_name              # Human-readable label (e.g. "anchor_corner_1")

float64 range                   # Measured distance in meters (ToA / Ranging)
float32 rssi                    # Signal strength in dBm (for NLOS detection)
float32 range_variance          # Measurement noise variance (m^2)

geometry_msgs/Point anchor_pose # Known or ground-truth anchor location (x, y, z)
