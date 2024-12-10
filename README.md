
# OptiTrack Motion Capture Setup Guide with VRPN and TurtleBot3 on ROS Melodic and WSL (Windows Subsystem for Linux)

## Prerequisites
- **ROS Melodic** is already installed on your WSL machine.
- **OptiTrack Motive** is installed and running on your Windows machine.
- **TurtleBot3** packages are installed and configured.

## Steps to Configure and Connect VRPN with TurtleBot3 Using Real OptiTrack Data

### 1. Install the `vrpn_client_ros` Package

Install the VRPN Client ROS Package:  
Open a terminal in WSL and run:

```bash
sudo apt-get update
sudo apt-get install ros-melodic-vrpn-client-ros
```

### 2. Configure Motive (on Windows)

Enable VRPN Streaming:
- Open OptiTrack Motive on your Windows machine.
- Go to **Data Streaming Settings** and enable **VRPN**.
- Copy the port number (default is 3883) from Motive for later use in the ROS launch file.

Check Firewall Settings:
- Allow traffic on the VRPN port in your Windows firewall.
- Go to **Windows Firewall with Advanced Security** > **Inbound Rules**, and create a new rule allowing UDP traffic on the VRPN port.

### 3. Set Up the TurtleBot3 and OptiTrack Integration

Now, we will configure the TurtleBot3 to be controlled based on the motion capture data streamed from OptiTrack via VRPN.

1. **Create a Launch File for TurtleBot3 Using VRPN**:

Create a new launch file in your Catkin workspace:

```bash
mkdir -p ~/turtlebot3_ws/src/turtlebot3/launch
nano ~/turtlebot3_ws/src/turtlebot3/launch/turtlebot3_vrpn.launch
```

2. **Paste the Following Content into the File**:

```xml
<launch>
  <!-- Original TurtleBot3 Parameters -->
  <arg name="model" default="burger" doc="model type [burger, waffle, waffle_pi]"/>  
  <arg name="x_pos" default="-2.0"/>
  <arg name="y_pos" default="-0.5"/>
  <arg name="z_pos" default="0.0"/>

  <!-- Launch Gazebo with Empty World -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn TurtleBot3 in Gazebo -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />

  <!-- VRPN Client to Receive Real OptiTrack Data -->
  <node pkg="vrpn_client_ros" type="vrpn_client_node" name="vrpn_client_node" output="screen">
    <rosparam subst_value="true">
      server: "Your_Windows_IP"  <!-- Replace with your Windows IP address -->
      port: 3883
      update_frequency: 100.0
      frame_id: "world"
      use_server_time: false
      broadcast_tf: true
      trackers:
        - TestBody  <!-- Replace with your actual rigid body name in Motive -->
    </rosparam>
  </node>

  <!-- Static Transform from world to optitrack -->
  <node pkg="tf" type="static_transform_publisher" name="world_to_optitrack" args="0 0 0 0 0 0 world optitrack 100" />

  <!-- Static Transform from TestBody (OptiTrack) to base_link -->
  <node pkg="tf" type="static_transform_publisher" name="testbody_to_base_link" 
        args="0 0 0 0 0 0 /TestBody /base_link 100" />

  <!-- Disable odom transform publication by TurtleBot -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
    <param name="publish_frequency" value="50.0" />
    <param name="use_sim_time" value="true" />
    <remap from="/odom" to="/fake_odom" />
  </node>
</launch>
```

Replace `"Your_Windows_IP"` with your actual Windows IP (use `ipconfig` to find it).  
Replace `"TestBody"` with the actual name of the rigid body being tracked by Motive.

3. **Build and Source the Workspace**:

After creating the launch file, build and source your workspace:

```bash
cd ~/turtlebot3_ws
catkin_make
source devel/setup.bash
```

### 4. Important Note for WSL Users

When using WSL, do **not** use `127.0.0.1` for the VRPN server address. The IP `127.0.0.1` refers to the WSL environment and not your Windows machine. Instead, use the IP address of your Windows machine (e.g., `10.8.2.136`), which you can find by running `ipconfig` in a Windows command prompt.

### 5. Test the Setup

1. **Start Motive** on your Windows machine and ensure the VRPN server is running.

2. **Launch the TurtleBot3 and VRPN Setup** in WSL:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3 turtlebot3_vrpn.launch
```

### 6. Verify the Setup in ROS

1. **Check for VRPN Topics**:

Verify that the VRPN topics are available in ROS by running:

```bash
rostopic list
```

You should see topics like `/vrpn/TestBody/pose`.

2. **Inspect the Pose Data**:

Inspect the pose data of the tracked body from OptiTrack:

```bash
rostopic echo /vrpn/TestBody/pose
```

3. **Visualize the Setup in RViz**:

Open RViz by running:

```bash
rviz
```

In RViz, follow these steps:
- Click the **"Add"** button in the lower-left corner.
- Select **"By Topic"** and expand the `/vrpn` topic.
- Choose the **"Pose"** option under `/vrpn/TestBody/pose` to visualize the tracked pose.
- Add a **RobotModel** display to visualize the TurtleBot3 model.
- Set the **Fixed Frame** to `world`.

### Troubleshooting

1. **Connection Issues**: Ensure that Motive and ROS can communicate over the same network using the correct IP address of the Windows machine.
2. **Firewall Issues**: Ensure that firewall rules allow traffic over the VRPN port (default `3883`).
3. **Network Configuration**: If using WSL, use your Windows IP (not `127.0.0.1`) for the VRPN server address.

With these steps, you should now be able to control and visualize TurtleBot3 using real-time OptiTrack motion capture data streamed over VRPN in ROS Melodic.
