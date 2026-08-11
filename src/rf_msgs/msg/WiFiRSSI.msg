# Custom ROS 2 Message for Wi-Fi RSSI RF Sensing
# Provides Received Signal Strength Indicator (RSSI) from Wi-Fi Access Points

std_msgs/Header header          # Timestamp and frame_id (e.g. wifi_antenna_link)

string bssid                    # Access Point MAC Address / BSSID (e.g. "AA:BB:CC:DD:EE:01")
string ssid                     # Human readable SSID label (e.g. "Research_AP_01")
uint32 ap_id                    # Access Point ID (1, 2, 3...)

float32 rssi                    # Signal strength in dBm (-30 dBm to -95 dBm)
float32 path_loss_exponent      # Environment path loss exponent n (typically 2.0 to 3.0)
float32 rssi_at_1m              # Reference RSSI P0 at 1 meter (-40.0 dBm)

geometry_msgs/Point ap_location # Known Access Point coordinate (x, y, z)
