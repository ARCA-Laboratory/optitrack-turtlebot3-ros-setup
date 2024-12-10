# ROS2 Humble Setup Guide with Docker, rosbridge_websocket, and ROSbot Simulation

## Prerequisites

- **Docker** is installed and running on your system.
- **Visual Studio Code (VSCode)** is installed, along with the Remote Development extension.

---

## Steps to Set Up ROS2 Humble and Associated Tools

### Step 1: Set Up ROS2 Humble in Docker

1. Open Visual Studio Code (VSCode).
2. Install the **Remote Development** extension.
3. Open the bottom-left menu (`><`) in VSCode and select **New Dev Container**.
4. Search for `ros2` and select the **ROS2 Workspace BrunoB81HK** option.
5. Once the setup is complete, validate the ROS2 installation by running the following command in the terminal:

   ```bash
   ls /opt/ros
   ```

   You should see the available ROS2 distributions, including `humble`.

---

### Step 2: Install and Configure `rosbridge_websocket`

1. **Install the `rosbridge_server` Package**:

   ```bash
   sudo apt update && sudo apt install -y ros-humble-rosbridge-server
   ```

2. **Verify the Installation**:

   ```bash
   ros2 pkg list | grep rosbridge_server
   ```

   Ensure `rosbridge_server` is listed in the output.

3. **Launch the WebSocket Server**:

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

   This starts the WebSocket server on the default port (9090), allowing communication between ROS2 and external systems.

4. **Optional - Run Manually Without Launch File**:

   ```bash
   ros2 run rosbridge_server rosbridge_websocket
   ```

---

### Step 3: Install and Run ROSbot Simulation

1. **Set Up the ROSbot Gazebo Simulation**:

   Clone and build the ROSbot repository:

   ```bash
   git clone https://github.com/husarion/rosbot_ros.git
   cd rosbot_ros
   colcon build
   ```

2. **Run the ROSbot Simulation**:

   Launch the simulation in Gazebo:

   ```bash
   source install/setup.bash
   ros2 launch rosbot_gazebo simulation.launch.py
   ```

3. **Simulate Multiple Robots**:

   To launch multiple robots in the simulation, use the following command:

   ```bash
   ros2 launch rosbot_gazebo simulation.launch.py robots:="robot1={x: 0.0, y: 0.0, yaw: 0.0}; robot2={x: 2.0, y: 0.0, yaw: 1.57};"
   ```

---

## Testing and Verification

1. **Verify ROS2 is Running in the Container**:

   Check that ROS2 commands work by listing installed packages:

   ```bash
   ros2 pkg list
   ```

2. **Test the `rosbridge_websocket` Server**:

   Confirm that the WebSocket server is running by connecting to port 9090 with a WebSocket client.

3. **Check ROSbot Simulation**:

   Use Rviz to visualize the ROSbot simulation:
   - Launch Rviz:
     ```bash
     rviz
     ```
   - Add the **RobotModel** display to visualize the ROSbot.
   - Set the **Fixed Frame** to `world`.

---

## Troubleshooting

1. **Docker Issues**: Ensure Docker is installed and running properly on your system. Test by running `docker --version`.
2. **Network Configuration**: If you're connecting to the WebSocket server from an external machine, ensure the port (9090) is open in your firewall.
3. **Simulation Performance**: For large-scale simulations with multiple robots, consider increasing the resources allocated to your Docker container.

By following these steps, you will have a fully operational ROS2 Humble setup in Docker with `rosbridge_websocket` and ROSbot simulation running in Gazebo.
