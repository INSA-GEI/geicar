# This file describes the steps to configure the Jetson Nano board, from the OS to the ROS installation

## 1. Jetson configuration from the SD card image
**An image of the SD card with all the necessary configurations and the initial ROS nodes is available here : .....**

1. Flash the image on your SD Card (you can use etcher)
2. Insert the SD card in the Raspberry and Power ON the board (login = pi, password = geicar)
3. You will then have to change the ROS_ID at the end of the ~/.bashrc file in the docker container, with XX the number of the car (1, 2...): 
```sh
sudo docker start -ai ros-humble
sudo nano ~/.bashrc
```
Then change : 
```sh
export ROS_DOMAIN_ID = XX
```

4. Reboot :
```sh
sudo reboot
```


## 2. Jetson configuration from an empty SD card
## OS Installation
OS installation instructions for the Nvidia jetson board are available here: "https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit". Follow these instructions with :\
login = jetson\
hostname = geicar\
password = geicar

## General Updates and tools installation **(internet connection required)**

```sh
sudo apt update
sudo apt install nano
apt install net-tools
sudo apt install iputils-ping
sudo apt-get install ufw
```

## Firewall configuration :
1. Disable IPV6 in "/etc/default/ufw" : change IPV6=yes to IPV6=no 

2. Configure firewall rules :
```sh
sudo ufw allow from 192.168.1.1
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw disable
sudo ufw enable
```

3. Loopback multicast configuration :
```sh
route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
ifconfig lo multicast
```

4. Eth0 multicast configuration :
```sh
route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
ifconfig eth0 multicast
```

## Optionnal - Installation of ROS2 Dashing (if you want to use rviz2) **(internet connection required)**
The procedure to install ros2 dashing can be found here : "https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html"
**You need to install the "desktop" version : "ros-dashing-desktop"**

## Installation of ROS2 Humble with a docker container **(internet connection required)**

Source : "https://hub.docker.com/r/arm64v8/ros/tags"

1. Connect the LIDAR and the CAMERA to the Jetson board (USB)

2. Configure USB device :
```sh
sudo chmod 666 /dev/ttyUSB0 
```

3. Pull the ros-humble docker image :
```sh
sudo docker pull arm64v8/ros:humble-ros-base 
```

4. Create a ros-humble docker container :
```sh
sudo docker run -it --device=/dev/ttyUSB0 --device=/dev/video0 --net=host --name=ros-humble arm64v8/ros:humble-ros-base 
```

## Installation of the ros2_ws **(internet connection required)**

1. Install v4l-utils :
```sh
sudo apt-get update 
sudo apt-get install v4l-utils
```

2. Go into the docker container "ros-humble" :
```sh
docker start -ai ros-humble
```

3. Copy the [ros2_ws](./ros2_ws) to ~/ros2_ws 

4. Install v4l-utils :
```sh
sudo apt-get update 
sudo apt-get install v4l-utils
```

5. Install dependencies and source the ROS environment
```sh
cd ~/ros2_ws
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

6. You can check the installation with :
```sh
ros2 launch geicar_start_jetson geicar.jetson.launch.py
```
If the installation was successful, the LIDAR and the CAMERA should be running.


## Source and configure the ROS environment

1. Go into the docker container ros-humble :
```sh
sudo docker start -ai ros-humble
```

2. Source the ROS environment :\
**Replace XX by the car number (1,2,3,4 ...). Example : "export ROS_DOMAIN_ID=1" for the car n°1**

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=XX" >> ~/.bashrc  
```
