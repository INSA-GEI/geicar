# This directory is dedicated to the simulation with the CARLA simulator

The basics for running a simulation with CARLA are included in the car environment ("simulation" package on the raspberry). However, this needs to be largely completed in order to achieve an interesting simulation.

The Carla simulator (on the PC) can communicate with the ROS environment of the car if they are **on the same network**

The "**carla_msgs**" package contains the ROS messages used by the CARLA simulator.\
The "**simulation**" node retrieves the joystick commands and publishes them on the Carla topics. It only shows how to link the car to the simulator, without making a real simulation.

## CARLA installation on the PC

**1. Install CARLA Simulator**
https://carla.readthedocs.io/en/0.9.12/start_quickstart/ 

To install CARLA, run :
```sh
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
sudo apt-get update # Update the Debian package index
apt-cache madison carla-simulator # List the available versions of Carla
sudo apt-get install carla-simulator=0.9.13 

sudo apt-get install libomp-dev
sudo apt install python3-opencv
export CARLA_ROOT=/opt/carla-simulator/
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
```

**2. Instal carla-ros-bridge**

carla-ros-bridge is the link between the Carla simulator and the ROS environment (messages, topics ...)

The installation procedure of carla-ros-bridge is available here: https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/

Then, modify the file "/home/adminlocal/carla-ros-bridge/src/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION" according to the installed CARLA version

**3. Network configurations (on the PC)**

* Connect the pc to the same network as the raspberry : Test the connection with a ping
* Use the same ROS_ID as the car (1 for car n°1 ...) :
```sh
export ROS_DOMAIN_ID = XX
```

* Allow multicast :
```sh
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```

* Configure firewall:
```sh
sudo ufw allow from "@IP_Raspberry"
```

**4. Launch the carla simulator (on the PC)**

```sh
cd /opt/carla-simulator/
./CarlaUE4.sh
```

**5. Launch the carla-ros-bridge**
```sh
cd ~/carla-ros-bridge
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py timeout:=30
```
This should make a vehicle appear in the carla simulator

**6. Start the ros nodes in the car**
```sh
ros2 launch geicar_start geicar.launch.py
ros2 run simulation simulation_node
```
**You can now control the car in the simulator from the joystick, through the raspberry firmware**
