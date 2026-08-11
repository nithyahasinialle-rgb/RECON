# Project RECON: Multi-Sensor 3D Time-of-Flight Array & RF SLAM System
**Technical System Architecture Specification & Official User Guide**

---

## 1. Executive System Overview

**Project RECON** is a high-precision, multi-sensor autonomous mobile robot platform engineered for simultaneous 3D indoor spatial mapping, RF signal trilateration, and hazard navigation in GPS-denied environments.

The system fuses four primary sensor modalities:
1. **3D Multi-Zone Time-of-Flight (ToF) Infrared Laser Arrays**: Four ST VL53L5CX $8\times 8$ zone sensors (144 3D spatial rays total) providing dense 3D point cloud surface mapping.
2. **RF / Wi-Fi Signal Analytics**: Log-Distance Path Loss RSSI/CSI trilateration from 3 fixed Access Point (AP) beacons for absolute global positioning ($x, y$).
3. **Inertial Measurement Unit (IMU)**: BNO085 / MPU6050 6-DOF / 9-DOF IMU providing high-frequency ($50\text{ Hz}$) angular velocity and quaternion yaw orientation lock ($\theta$).
4. **Wheel Odometry**: Differential drive encoder kinematics measuring linear velocity ($v_x$).

All sensor streams are fused inside a custom **C++ Extended Kalman Filter (EKF)** running at $20\text{ Hz}$ on an **NVIDIA Jetson Orin Nano**, outputting state estimates to a **Three.js WebGL 3D Visualization Dashboard** and ROS 2 RViz.

---

## 2. Quick Start Execution Commands

### Prerequisites
* **OS**: Ubuntu 22.04 LTS / ROS 2 Humble
* **Hardware / Remote**: NVIDIA Jetson Orin Nano (accessible via Tailscale / LAN at `100.67.245.97`)
* **Dependencies**: ROS 2 Humble Desktop (`ros-humble-desktop`), `colcon`, `eigen3`, `three.js`, `python3-psutil`

---

### Step 1: Launch Master Simulation Engine (Terminal 1)

```bash
cd ~/downloads/RECON

# 1. Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# 2. Build workspace packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Source local build setup
source install/setup.bash

# 4. Launch Master Simulation (Gazebo, EKF Node, ToF Raycaster & RViz)
ros2 launch rover_gazebo simulation.launch.py
```

---

### Step 2: Launch 3D Web Dashboard Server (Terminal 2)

```bash
cd ~/downloads/RECON/web_dashboard
python3 -m http.server 8000
```

Now open your web browser on your laptop to:

👉 **`http://100.67.245.97:8000`**
*(or double-click `C:\Users\nithy\Downloads\RECON\web_dashboard\index.html` on your laptop!)*

---

### Step 3: Run Autonomous Evaluation Benchmark (Terminal 3)

```bash
cd ~/downloads/RECON
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run 120-second autonomous indoor office trail
python3 benchmarks/autonomous_coverage_benchmark.py 120
```

*Results are recorded automatically into `IEEE_Indoor_Lab_Benchmark_Results.csv`.*

---

### Step 4: Manual Keyboard Teleoperation (Optional - Terminal 4)

```bash
cd ~/downloads/RECON
source /opt/ros/humble/setup.bash
source install/setup.bash

# Keyboard Teleop Controller (WASD keys to drive)
ros2 run rover_control tof_teleop
```

---

## 3. System Architecture & Data Flow

```text
[ Hardware Layer (ESP32 & Sensors) ]
 ├── 4x ST VL53L5CX 8x8 ToF Sensors (Front, Left, Right, Rear)
 ├── MPU6050 / BNO085 IMU Sensor (50 Hz Yaw Lock)
 └── TCA9548A I2C Multiplexer
           │
           ▼ (micro-ROS / Wi-Fi UDP Stream over Tailscale)
[ Jetson Orin Nano Processing Pipeline ]
 ├── EKF Multisensor Fusion Core (C++ / Eigen3 @ 20 Hz)
 ├── 3D ToF Spatial Raycaster (144 3D Rays / Permanent Red Hit Dots)
 └── Wi-Fi RSSI / CSI Path Loss Trilateration Engine
           │
           ▼
[ User Interface & Visualization Layer ]
 ├── Three.js WebGL 3D Dashboard (HTTP Port 8000)
 └── Native ROS 2 RViz2 GUI Configuration
```

---

## 4. Mathematical Foundations & EKF Formulations

### 4.1 State Vector Definition
The system state at time step $k$ is represented by a 5-dimensional state vector $\mathbf{x}_k \in \mathbb{R}^5$:

$$\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \\ v_{x, k} \\ \omega_{z, k} \end{bmatrix}$$

Where $x_k, y_k$ represent world position coordinates ($\text{meters}$), $\theta_k$ represents heading yaw ($\text{radians}$), $v_{x, k}$ represents linear velocity ($\text{m/s}$), and $\omega_{z, k}$ represents angular yaw velocity ($\text{rad/s}$).

---

### 4.2 Wi-Fi Path Loss & Measurement Jacobian Matrix

Wi-Fi RSSI received from Access Point $i$ at known position $(x_i, y_i)$ is converted to a range measurement using the Log-Distance Path Loss model:

$$d_{\text{meas}, i} = d_0 \cdot 10^{\left(\frac{P_0 - \text{RSSI}_i}{10 \cdot n}\right)}$$

The 1x5 Measurement Jacobian $H_{\text{WiFi}, i} = \frac{\partial h_i}{\partial \mathbf{x}}$ is:

$$H_{\text{WiFi}, i} = \begin{bmatrix} \frac{x_k - x_i}{\hat{d}_i} & \frac{y_k - y_i}{\hat{d}_i} & 0 & 0 & 0 \end{bmatrix}$$

The EKF correction steps execute as:
1. **Innovation Residual**: $y_k = d_{\text{meas}, i} - h_i(\mathbf{x}_k^-)$
2. **Innovation Covariance**: $S_k = H_{\text{WiFi}, i} P_k^- H_{\text{WiFi}, i}^T + R_{\text{WiFi}}$
3. **Kalman Gain**: $K_k = P_k^- H_{\text{WiFi}, i}^T S_k^{-1}$
4. **State Update**: $\mathbf{x}_k^+ = \mathbf{x}_k^- + K_k \cdot y_k$
5. **Covariance Update**: $P_k^+ = (I - K_k H_{\text{WiFi}, i}) P_k^-$

---

## 5. Hardware Integration & Assembly Plan

```text
               +-------------------------------------------------+
               |             ESP32 Microcontroller               |
               |                                                 |
               | [GPIO 21: SDA] ----------+                     |
               | [GPIO 22: SCL] --------+ |                     |
               +------------------------|-|----------------------+
                                        | |
                                        v v
                       +-----------------------------------+
                       |    TCA9548A I2C Multiplexer       |
                       +-----------------------------------+
                         |       |       |       |       |
                         v       v       v       v       v
                       Channel 0 |       |       |    Channel 4
                         |     Channel 1 |       |       |
                         v       v     Channel 2 |       v
                      +-----+ +-----+    v     Channel 3+-----+
                      | ToF | | ToF |  +-----+   v      | IMU |
                      |Front| | Left|  | ToF | +-----+  |MPU  |
                      +-----+ +-----+  |Right| | ToF |  |6050 |
                                       +-----+ | Rear|  +-----+
                                               +-----+
```

---

## 6. Workspace Package Structure

```text
RECON/
├── benchmarks/
│   ├── autonomous_coverage_benchmark.py   # IEEE evaluation benchmark trail & CSV exporter
│   └── jetson_benchmark.py                # Hardware CPU/RAM resource monitor
├── src/
│   ├── rf_msgs/                           # Custom ROS 2 message definitions (WiFiRSSI, ToFRange)
│   ├── rover_control/                     # Keyboard teleop & obstacle collision avoidance
│   ├── rover_description/                 # URDF 3D robot model & Xacro files
│   ├── rover_gazebo/                      # Gazebo world (rf_arena.world) & launch files
│   ├── sensor_fusion/                     # C++ EKF multisensor fusion engine (ekf_fusion_node.cpp)
│   ├── tof_sensor_sim/                    # 3D ToF spatial raycaster (tof_publisher_node.py)
│   └── wifi_rf_sim/                       # Wi-Fi RSSI/CSI beacon simulation node
├── web_dashboard/
│   ├── index.html                         # YouTube-style 3D WebGL scanner dashboard UI
│   └── app.js                             # Three.js 3D viewport & permanent point cloud engine
├── README.md                              # Official project specification & user manual
└── RECON_Technical_System_Specification.docx
```
