## Suction Grasping Pipeline (Suctionnet)

This workspace executes the suction grasping pipeline using **Suctionnet**.

### Prerequisites
- Please replace the provided **inference.py** in /suctionnet-baseline/neural_network
- Download **realsense-deeplabplus-RGBD** pretrained model of suctionnet and place it inside /suctionnet-baseline/neural_network/example_data/
- Edit **inference_command.sh** file to match your parameters.
- Here we use two relays: 
    - first one is for activating the suction gripper which is in serial connection with air flow to the suction gripper to turn on and off the supply.
    - second one is for data collection and it will be described under data collection pipeline

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
### Step 3: Activate the suction gripper
```bash
ros2 run relay_control relay_control
```
### Step 4: Activate the data collection pipeline

This section describes the integration of the Arduino-based touch sensing system with the ROS 2 pipeline.

- An **Arduino Mega 2560** is used to interface the touch sensor and establish a serial connection with the relay modules.

- The `suction_move_robot` node publishes `"SON"` and `"SOFF"` commands to the ROS 2 topic: /sensor_control.

- The `sensor_control` package acts as a **subscriber node**:

- It listens to the `/sensor_control` topic  

- Sends corresponding `"SON"` / `"SOFF"` commands to the Arduino via serial communication  

To run the sensor control node:
```bash
ros2 run sensor_control sensor_control
```
- The Arduino must run a separate PlatformIO project that:
   - Reads serial input of both ON / OFF and SON/SOFF
   - Controls the relay pin accordingly for suction gripper activation and data collection activation
   - It prints "START_LOGGING" / "STOP_LOGGING" messages to the serial monitor which will be used by publisher_anygrasp_suction.py

  Refer to the file: 

  ```bash
  main_suction.cpp
  ```
- Sensor data collection is handled by the Python script publisher_anygrasp_suction.py, which:

   - Monitors the serial output from Arduino
   - Starts/stops logging based on "START_LOGGING" / "STOP_LOGGING" messages

  To run the script:
  ```bash
  python3 /path_to_file_location/publisher_anygrasp_suction.py
  ```
