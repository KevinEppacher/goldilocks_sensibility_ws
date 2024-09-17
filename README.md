
# Docker Installation Guide

## Docker Installation Documentation:
You can follow the official Docker installation documentation for Ubuntu by visiting the following link:  
[Docker Install Guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

### Step-by-Step Installation:

1. Update your package list:
   ```bash
   sudo apt-get update
   ```

2. Install the required packages:
   ```bash
   sudo apt-get install ca-certificates curl
   ```

3. Add Docker's official GPG key:
   ```bash
   sudo install -m 0755 -d /etc/apt/keyrings
   sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
   sudo chmod a+r /etc/apt/keyrings/docker.asc
   ```

4. Add the Docker repository to your Apt sources:
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   ```

5. Update your package list again:
   ```bash
   sudo apt-get update
   ```

6. Install Docker:
   ```bash
   sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```

7. Verify Docker installation by running the hello-world container:
   ```bash
   sudo docker run hello-world
   ```

## Docker Post-Installation Steps:
After installing Docker, you can follow these post-installation steps:  
[Docker Post-Install Guide](https://docs.docker.com/engine/install/linux-postinstall/)

1. Create the Docker group:
   ```bash
   sudo groupadd docker
   ```

2. Add your user to the Docker group:
   ```bash
   sudo usermod -aG docker $USER
   ```

3. Log out and log back in for the changes to take effect.

4. Start a new shell session:
   ```bash
   newgrp docker
   ```

5. Verify Docker is working without `sudo`:
   ```bash
   docker run hello-world
   ```

6. Enable Docker services to start on boot:
   ```bash
   sudo systemctl enable docker.service
   sudo systemctl enable containerd.service
   ```

---




# Goldilocks Sensibility Measurements

![sensbility_measurement_demo_1](docs/sensbility_measurement_demo_1.png)

![sensbility_measurement_demo_2](docs/sensbility_measurement_demo_2.png)

![sensbility_measurement_demo_3](docs/sensbility_measurement_demo_3.png)

![sensbility_measurement_demo_4](docs/sensbility_measurement_demo_4.png)


Connect Ethernet cable from UR to Laptop

start up UR

initialize UR

Make sure robot ip adress is: 192.168.1.101

open vsc

select workspace folde goldilocks_sensibility_ws

Either connect vsc with remote docker or install all dependencies on local laptop (Dependencies are in the Dockerfile and execute rosdep install ... )

open 3 termials, go to path of workspace and source the terminals source devel/setup.bash

now execute following commands in this order

roslaunch rosbridge_server rosbridge_tcp.launch

this launch file connects laptop with UR via tcp. In this case used for publishing the force torque sensor values




roslaunch tars_robot measure_sensitivity.launch

this launch file opens a hmi for controlling the current robot program and for teaching the measurement points

under the reiter Control Panel, the user is able to control the current robot program

- Setup Button sets the robot in freedrive mode. You can use this mode to teach the measurement points

- Begin External Control sets the robot in external control, which can be controlled via MoveIt RViz or MoveGroup

- Default settings stops both programms and stops the tcp communication. No program is selected



roslaunch tars_robot start_sensibility_measurement.launch 

when all measurement points are teached, and the user is in the Measurement Panel, then the robots starts the sensibility measurements

the sensibility measurement program parameters can be set here:

src/tars_robot/param/sensitivity_program_param.yaml


when the measurement program is finished, it should have saved csv files in this folder:

src/tars_robot/data

When you want to have plots of each measurement of each measurement set, the python scripts checks if any plots need to done. If in each folder is no plot, it automatically generates one

If you want to visualize a 3D representation of the measurement, then you need to connect the camera to the laptop (sidenote: either connect camera directly with laptop or when a extension cable is used, make sure the cable is externally powered via a docking station and the usb is connected on the fast usb port)

Note:
when you want to close the hmi window, then press first the default button and after that close the windows ord ctrl+C in the terminal. If you close the window while a program is running, the rosbridge connecion dies and shouts out a error message. The user has to stop the program on the ur10, by going on the program button and selecting stop program on the control panel.

# Hardware setup

make sure that ...

the ft300-sensor is connected properly to the UR10 (via USB)

the camera is mounted on the test finger gripper

Polyscope > 3.12.x

ros1 noetic devel is installed

# UR Robot Startup Guide

## Preparation:

1. Navigate to your Catkin workspace:
   ```bash
   cd ~/catkin_ws
   ```

2. Ensure your laptop IP is configured correctly:
   ```bash
   Laptop IP = 192.168.1.101
   ```

3. Update the system and install necessary ROS dependencies:
   ```bash
   sudo apt-get update
   rosdep install --from-paths src --ignore-src -y
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Launching the UR Robot:

1. Run the calibration correction launch file:
   ```bash
   roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.102 target_filename:="${HOME}/my_robot_calibration.yaml"
   ```

2. Bring up the UR10 robot with the driver:
   ```bash
   roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.102 kinematics_config:=/home/kevin/catkin_ws/src/ur_calibration/ex-ur5_calibration.yaml
   ```

3. Start the MoveIt planning and execution:
   ```bash
   roslaunch ur10_moveit_config moveit_planning_execution.launch limited:=true
   ```

4. Launch the MoveIt Rviz interface:
   ```bash
   roslaunch ur10_moveit_config moveit_rviz.launch config:=true
   ```

---

# Camera Startup

Launch the RealSense camera and point cloud:
```bash
roslaunch realsense2_camera demo_pointcloud.launch
```

# Starting Program on Laptop

Launch ROSBridge Server for TCP communication:
```bash
roslaunch rosbridge_server rosbridge_tcp.launch
```

# Gazebo Simulation

To start the Gazebo simulation for the UR robot, run:
```bash
roslaunch tars_robot gazebo_sim.launch
```

# UR Robot Service Calls

1. Load a specific program on the UR robot:
   ```bash
   rosservice call /ur_hardware_interface/dashboard/load_program "{filename: '/programs/Sensitivity_Measurement/2024_07_29_External_Control.urp'}"
   ```

2. Start the loaded program:
   ```bash
   rosservice call /ur_hardware_interface/dashboard/play "{}"
   ```

3. Stop the running program:
   ```bash
   rosservice call /ur_hardware_interface/dashboard/stop "{}"
   ```

---

**Note:** These instructions are designed for a manual setup, but they can be automated by integrating them into a Dockerfile for ease of deployment in the future.