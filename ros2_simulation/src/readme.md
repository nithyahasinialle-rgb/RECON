# ROS 2 Spatial Mapping & SLAM via Wi-Fi CSI & 6-DoF IMU

## 📖 Abstract
This repository contains a full ROS 2 Humble implementation for converting an ESP32 and a standard 6-DoF IMU into a real-time RF spatial mapping and human presence detection system. By analyzing the Channel State Information (CSI) of ambient Wi-Fi packets, the system performs Fast Fourier Transforms (IFFT) to extract Time-of-Flight (ToF) distances to walls, objects, and humans. It visualizes this data in full 3D via Foxglove Studio, mimicking the behavior of a Submarine Sonar or Aviation Radar.

---

## ⚙️ Hardware Architecture

* **Host Brain:** Raspberry Pi 4 (running Dockerized ROS 2 Humble)
* **RF Receiver:** ESP32 (Extracting 802.11n Wi-Fi CSI payloads via Serial UART)
* **Inertial Sensor:** MPU6500 6-DoF IMU (Connected via I2C for orientation tracking)
* **Client Interface:** PC running Foxglove Studio connected via `rosbridge_websocket`

## 📌 Software Architecture Pipeline

```text
 [ESP32 CSI Sniffer]          [MPU6500 6-DoF IMU]
      (UART / Serial)               (I2C-1)
             |                         |
             V                         V
   +------------------------------------------+
   |            sensor_hub_node               |
   | (Translates raw bytes to ROS 2 Messages) |
   +--------------------+---------------------+
                        |
        +---------------+---------------+
        | /wifi/csi (CSIData)           | /imu/data_raw (Imu)
        V                               V
   +------------------------------------------+
   |           recon_slam_node                |
   |  * Startup Gyro Bias Calibration         |
   |  * CSI Phase Sanitization & Clutter Sub. |
   |  * Zero-Padded IFFT Range Profile (CIR)  |
   |  * Radar Fan PointCloud2 Generation      |
   |  * CSI Subcarrier Variance Vitals Engine |
   +--------------------+---------------------+
                        |
       +----------------+----------------+
       V                V                V
  /robot_pose      /map_points    /human_detection
 (PoseStamped)    (PointCloud2)       (Marker)
       |                |                |
       +----------------+----------------+
                        V
            +----------------------+
            |   Foxglove Studio    |
            |  (3D Visualization)  |
            +----------------------+
```

---

## 🧮 Mathematical Models & Simulation Parameters

The `recon_slam_node` utilizes a highly tuned set of parameters to bridge the gap between raw hardware signals and the ROS 2 3D simulation environment.

### 1. IMU Integration & Dead-Reckoning
* **Math:** $\theta_t = \theta_{t-1} + (\omega_{z} - \mu_{bias}) \cdot \Delta t$
* `CALIB_SAMPLES = 100`: The system aggregates the first 100 frames at boot to calculate the static gyroscope offset/bias ($\mu_{bias}$).
* `YAW_DEADBAND = 0.05`: A hard threshold (in rad/s). If $|\omega_z - \mu_{bias}| < 0.05$, the movement is discarded as electrical noise to prevent infinite drift.

### 2. Time-of-Flight (ToF) & Spatial Mapping
* **Phase Detrending:** $\phi_{clean} = \phi_{raw} - (a \cdot k + b)$ eliminates timing offsets across orthogonal subcarriers.
* **IFFT Distance:** $d = \frac{\text{peak\_idx} \cdot c}{2 \cdot \text{BW} \cdot N_{pad}}$ 
* `CSI_BG_ALPHA = 0.95`: The $\alpha$ parameter for the Exponential Moving Average (EMA) filter. A high value means the background clutter updates slowly, perfectly isolating fast-moving multipath changes.
* `CSI_PEAK_THRESH = 0.35`: The IFFT magnitude must be at least 35% of the global maximum to be registered as a physical wall.
* `CSI_PEAK_DIST = 8`: The minimum bin separation ($\approx 2.3m$) required to register two distinct walls, preventing multipath smearing.
* `CSI_MIN_DIST_M = 0.5` / `CSI_MAX_DIST_M = 12.0`: Limits that eliminate the massive near-field DC spike (internal board interference) and extreme far-field noise.

### 3. Radar Fan Extrusion (PointCloud2 Simulation)
To simulate a solid 3D wall from a single-antenna 1D distance measurement, we project the distance along a probability arc and extrude it vertically.
* **Probability Projection:** $x = (d + \mathcal{N}(0, \sigma_d)) \cdot \cos(\theta_{imu} + \theta_{fan} + \mathcal{N}(0, \sigma_\theta))$
* `CSI_FAN_HALF_ANGLE = 0.4`: The fan spread in radians ($\pm 23^\circ$ around the IMU vector).
* `CSI_FAN_RAYS = 18`: The density of the probability arc (18 rays per wall hit).
* `CSI_DIST_NOISE = 0.15` / `CSI_ANG_NOISE = 0.05`: Gaussian noise variance ($\sigma_d, \sigma_\theta$). This adds volumetric "thickness" to the PointCloud, turning a mathematically invisible 1-pixel line into a visibly thick, realistic radar cloud in Foxglove.
* **Z-Extrusion:** Iterates across $Z \in [0.0, 0.4, 0.8, 1.2, 1.6, 2.0]$ meters to build a solid 3D structural pillar.

### 4. Human Micro-Doppler Variance
* **Math:** $\sigma^2 = \frac{1}{N} \sum_{i=1}^N (x_i - \mu)^2$ applied to the amplitude of subcarrier 26.
* `HUMAN_BUFFER_LEN = 40`: Evaluates the rolling variance over the last 40 frames.
* `HUMAN_ENERGY_THRESH = 80.0`: The critical trigger threshold. Background room noise normally fluctuates between 13.0 and 22.0. A breathing or moving human causes the subcarrier amplitude variance to spike above 80.0, triggering the detection state.

---

## 🚀 Setup & Execution

### 1. Build the Workspace
Ensure you are running inside a ROS 2 Humble environment (or Docker container) on your Raspberry Pi:
```bash
source /opt/ros/humble/setup.bash
cd ~/recon_ws
colcon build
source install/setup.bash
```

### 2. Launch the Brain
Launch the system with logging suppressed to `WARN` to prevent raw serial data from overwhelming the SSH terminal:
```bash
# Launch the Foxglove WebSocket Bridge for Visualization
ros2 run foxglove_bridge foxglove_bridge > /dev/null 2>&1 &

# Launch Hardware Ingest (Reads serial/I2C and publishes raw topics)
ros2 run recon_brain sensor_hub --ros-args --log-level WARN > /dev/null 2>&1 &

# Launch DSP, SLAM, and Radar Mapping
ros2 run recon_brain recon_slam --ros-args --log-level WARN > /dev/null 2>&1 &
```
> **⚠️ CRITICAL:** Leave the hardware perfectly stationary on a flat surface for 1 full second immediately after launching `recon_slam`. The IMU requires this static period to calculate its zero-bias reference.

### 3. Connect Visualization
1. Open Foxglove Studio on a client PC on the same Wi-Fi network.
2. Connect to `ws://<RASPBERRY_PI_IP>:8765`.
3. Set the Global Display Frame to `odom`.
4. Enable the following topics:
   - `/robot_pose` (Set to 3D Arrow)
   - `/map_points` (Set Decay Time to `5000` to accumulate room history)
   - `/human_detection` (Displays the breathing/activity indicator)
