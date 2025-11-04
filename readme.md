# ROS2 Humble Example — Touch Sensor Data Reading

This package provides examples for **ROS2 Humble** users to read data from the **Touch Tronix tactile sensor**.  
Tested on **Ubuntu 22.04**.

---

## 🧩 Prepare Environment

### 1. Create a Workspace
Create your ROS2 workspace (you can change the name if needed):

```
mkdir -p ~/sensor_ws/src
```


2. Add the Package

Place the sensor package under the src folder:
```
cd ~/sensor_ws/src
```
# Copy or clone the package here

3. Build the Workspace

Go back to your workspace root and build the package:
```
cd ~/sensor_ws
colcon build --symlink-install
```
🚀 Run the Code
Terminal 1 — Run the Publisher Node

Source the workspace and run the sensor publisher node:
```
source install/setup.sh
ros2 run sensor_pkg sensor_pub --ros-args --params-file src/sensor_pkg/config/sensor_params.yaml
```
Terminal 2 — Run the Subscriber Node

Open another terminal, source the workspace again, and run the sensor subscriber node:
```
source install/setup.sh
ros2 run sensor_pkg sensor_sub
```


📄 Example Folder Structure
```
sensor_ws/
├── build/
├── install/
├── log/
└── src/
    └── sensor_pkg/
        ├── config/
        │   └── sensor_params.yaml
        ├── launch/
        ├── src/
        │   ├── sensor_pub.cpp
        │   └── sensor_sub.cpp
        ├── package.xml
        └── CMakeLists.txt
```

✅ Tip:

Ensure your tactile sensor is properly connected via USB or serial port before launching.

You can modify parameters (e.g., serial port, baud rate) in config/sensor_params.yaml.

To check your serial device name, run:

```
ls /dev/tty*
```

Tested Platform:

✅ Ubuntu 22.04

✅ ROS2 Humble Hawksbill

Dependencies:

rclpy

serial

std_msgs

sensor_msgs