## Suction Grasping Pipeline (Suctionnet)

This workspace executes the suction grasping pipeline using **Suctionnet**.

### Prerequisites

Before running the suction grasping pipeline, complete the following steps:

- Replace the provided `inference.py` file in:
  ```bash
  /suctionnet-baseline/neural_network
  ```
- Download [realsense-deeplabplus-RGBD](https://drive.google.com/file/d/18TbctdhpNXEKLYDWFzI9cT1Wnhe-tn9h/view) pretrained model of suctionnet and place it inside
  ```bash  
  /suctionnet-baseline/neural_network/example_data/
  ```
- Edit **inference_command.sh** file located in /suctionnet_benchmark_ws/suction_grasp/suction_grasp according to your system parameters.
- This setup uses two relay modules:
    - **Relay 1**: Controls the suction gripper by turning the air supply on and off through serial communication.
    - **Relay 2**: Supports the data collection process. This is described in the data collection pipeline section.

---

### Build the workspace
```bash
cd suctionnet_benchmark_ws
colcon build
```
### Step 1: Source the environment
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```
### Step 2: Launch the system (UR driver, MoveIt2, RealSense D415)
```bash
ros2 launch static_tf combined_launch.py
```
### Step 3: Activate Suction Gripper and Data Collection Pipeline

This section describes both suction gripper activation and sensor-based data logging using a single ROS 2 node.

- An **Arduino Mega 2560** is used to interface the touch sensor and establish a serial connection with the relay modules.

- The `suction_move_robot` node publishes `"SON"` / `"SOFF"` and `"ON"` / `"OFF"` commands to the ROS 2 topics: /sensor_control and /relay _control respectively.

- The `sensor_control` package acts as a **subscriber node**:

    - Listens to the `/sensor_control` topic and sends corresponding `"SON"` / `"SOFF"` commands to the Arduino via serial communication
    - Listens to the `/relay_control` topic and sends corresponding `"ON"` / `"OFF"` commands to the Arduino via serial communication

To run the sensor control node:
```bash
ros2 run sensor_control sensor_control
```
- The Arduino must run a separate PlatformIO project that:
   - Reads serial input of both ON/OFF and SON/SOFF
   - Controls the relay pin accordingly for suction gripper activation and data collection activation
   - It prints "START_LOGGING" / "STOP_LOGGING" messages to the serial monitor which will be used by sensor_control node again for data logging.

  Refer to the file: 

  ```bash
  main_suction.cpp
  ```

