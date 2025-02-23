# 🤖 Autonomous Mobile Robot (AMR) using ROS2

## 📌 Introduction  
The **Autonomous Mobile Robot (AMR)** is a self-navigating robotic system that utilizes **ROS2 (Robot Operating System 2)** for real-time control, localization, and navigation. Unlike AGVs, AMRs can dynamically **avoid obstacles, re-plan paths**, and operate efficiently in unstructured environments. This robot is useful in **warehouses, industrial automation, and smart factories**.

---

## 🌟 Features  
- **ROS2-based Control** – Uses ROS2 for a modular and scalable architecture.  
- **Autonomous Navigation** – Implements SLAM and path planning using the **Nav2 stack**.  
- **Obstacle Avoidance** – Utilizes **LiDAR, Ultrasonic Sensors, and Depth Cameras**.  
- **Real-time Mapping** – Implements **SLAM (Simultaneous Localization and Mapping)**.  
- **Wireless Monitoring** – Remote control and live data streaming.  
- **Multi-Robot Coordination** – Supports fleet management.  

---

## 🔩 Components Used  

### **1️⃣ Hardware Components**  
- **Processor**: Raspberry Pi 4 / NVIDIA Jetson Nano / Intel NUC  
- **Motor Driver**: L298N / TB6612FNG / ODrive (for high-power motors)  
- **Motors**: DC Motors with Encoders / Servo Motors  
- **Sensors**:  
  - **LiDAR Models (For SLAM & Obstacle Detection)**  
    - **RPLidar A1/A2/A3** (Affordable, 360° scanning)  
    - **Hokuyo URG-04LX / UST-10LX** (Compact, indoor mapping)  
    - **Velodyne VLP-16 / Puck** (3D LiDAR, outdoor navigation)  
    - **Ouster OS1 / OS2** (High-resolution, industrial-grade)  
  - **Ultrasonic Sensors** – For short-range obstacle avoidance  
  - **IMU (Inertial Measurement Unit)** – For precise localization  
  - **Depth Camera (Intel RealSense D435i / ZED2)** – For advanced perception  
- **Battery**: 12V Li-ion / 24V for high-power motors  
- **Communication Modules**:  
  - Wi-Fi (For ROS2 remote communication)  
  - Bluetooth (Optional for manual control)  

---

## 🛠️ Circuit Connection  

| Component  | Connection to Raspberry Pi / Jetson Nano |
|------------|----------------------------------|
| **LiDAR (RPLidar / Hokuyo / Velodyne / Ouster)** | UART / USB / Ethernet |
| **Motor Driver (L298N / ODrive)** | GPIO PWM Pins |
| **Ultrasonic Sensor** | Trigger & Echo Pins |
| **IMU Sensor** | I2C Communication |
| **Camera Module (Depth / RGBD)** | USB / CSI Interface |
| **Battery** | Power Supply (12V / 24V) |

---

## ⚙️ ROS2 Installation & Setup  

### **1️⃣ Install ROS2 (Foxy/Humble)**  
Follow the [official ROS2 installation guide](https://docs.ros.org/en/foxy/Installation.html).  

### **2️⃣ Setup the ROS2 Workspace**  
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### **3️⃣ Clone the AMR Repository**  
```bash
cd ~/ros2_ws/src
git clone my repository 
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### **4️⃣ Install Required Dependencies**  
```bash
sudo apt update
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-slam-toolbox ros-foxy-robot-localization
```

### **5️⃣ Running the AMR Navigation**  
#### **Start the ROS2 Core**  
```bash
ros2 launch amr_bringup bringup.launch.py
```
#### **Run SLAM for Mapping**  
```bash
ros2 launch amr_navigation slam.launch.py use_lidar:=true lidar_model:=rplidar
```
(Change `lidar_model` to `hokuyo`, `velodyne`, or `ouster` based on your hardware.)  

#### **Enable Obstacle Avoidance**  
```bash
```

---

## 📂 Project Structure  
```
📁 amr_ros2
│── 📁 src
│   ├── amr_control
│   ├── amr_navigation
│   ├── amr_sensors
│── 📁 launch
│   ├── bringup.launch.py
│   ├── navigation.launch.py
│── 📁 config
│   ├── params.yaml
│── 📁 scripts
│   ├── obstacle_avoidance.py
│── 📁 maps
│   ├── warehouse_map.pgm
│── 📁 Documentation
│   ├── README.md
```

---

## 🚀 Future Enhancements  
🔹 **AI-Based Object Recognition** – Integrating **YOLO** or **OpenCV** for advanced perception.  
🔹 **Multi-Robot Coordination** – Implementing **Swarm Robotics** for large-scale warehouse automation.  
🔹 **Cloud Connectivity** – Remote monitoring and control using **IoT**.  
🔹 **Upgrading to ROS2 Galactic / Humble** – For enhanced performance and new features.  

---

## 📜 License  
This project is open-source. Feel free to modify and improve!

