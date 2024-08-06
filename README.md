# Docker installation:
url: https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

#Add Docker s official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo docker run hello-world

docker post-install:
url: https://docs.docker.com/engine/install/linux-postinstall/

sudo groupadd docker

sudo usermod -aG docker $USER

#Log out and log back in so that your group membership is re-evaluated.

newgrp docker

docker run hello-world

sudo systemctl enable docker.service

sudo systemctl enable containerd.service



///////////////////////////////////////////////////////////////////////////////

UR Startup:

cd catkin_ws

Laptop Ip = 192.168.1.101

#temporäre Lösung --> muss automatisch im Dockerfile erfolgen --> alle packages herunterladen
sudo apt-get update

rosdep install --from-paths src --ignore-src -y
rosdep install --from-paths src --ignore-src -r -y

# diese commands bis hier ausführen

roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.102 target_filename:="${HOME}/my_robot_calibration.yaml"

roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.102 kinematics_config:=/home/kevin/catkin_ws/src/ur_calibration/ex-ur5_calibration.yaml

roslaunch ur10_moveit_config moveit_planning_execution.launch ĺimited:=true

roslaunch ur10_moveit_config moveit_rviz.launch config:=true


///////////////////////////////////////////////////////////////////////////////

Camera Startup:

roslaunch realsense2_camera demo_pointcloud.launch


Starting Program on Laptop:
roslaunch rosbridge_server rosbridge_tcp.launch


# Gazebo Simulation
```
roslaunch tars_robot gazeboi_sim.launch
```

rosservice call /ur_hardware_interface/dashboard/load_program "{filename: '/programs/Sensitivity_Measurement/2024_07_29_External_Control.urp'}"

rosservice call /ur_hardware_interface/dashboard/play "{}"

rosservice call /ur_hardware_interface/dashboard/stop "{}"