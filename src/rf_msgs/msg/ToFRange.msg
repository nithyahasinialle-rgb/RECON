# Custom ROS 2 Message for ST VL53L0X Time-of-Flight Distance Sensors
# Single-point laser distance measurement (0.03m to 2.0m range)

std_msgs/Header header          # Timestamp and frame_id (e.g. tof_front_link, tof_left_link...)

uint8 sensor_id                 # Sensor ID: 1=Front, 2=Left, 3=Right, 4=Rear
string sensor_label             # Label string ("front", "left", "right", "rear")

float32 distance                # Measured range distance in meters
float32 min_range               # Minimum operating range (0.03 m)
float32 max_range               # Maximum operating range (2.0 m)
float32 field_of_view           # Sensor field of view angle in radians (0.436 rad = 25 deg)

bool range_valid                # True if distance is within valid sensor bounds (0.03m - 2.0m)
bool hazard_warning             # True if obstacle is within critical proximity (< 0.25m)
