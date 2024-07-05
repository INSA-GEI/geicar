# This file describes the main steps to compile the ros2_ws on the Raspberry

**The raspberry Pi does not have the resources to compile the whole workspace at the same time.** It is better to compile the packages one by one (some packages can take 20 minutes to compile..)\
**In any case, the first package to compile is the "interfaces" package** because the other packages depend on it.\
**The compilation must ALWAYS be launched at the workspace root (~/ros2_ws)**

Syntax of the compilation command (```colcon build```):
* ```colcon build``` to compile all the workspace
* ```colcon build --packages-select "NameOfThePackage"```

1. Compilation of the "interfaces" package
```sh
cd ~/ros2_ws
colcon build --packages-select interfaces
```

2. Then, you can compile the other packages:
```sh
cd ~/ros2_ws
colcon build --packages-select carla_msgs
colcon build --packages-select can
colcon build --packages-select joystick
colcon build --packages-select car_control
colcon build --packages-select imu_tools
colcon build --packages-select imu_filter_madgwick
colcon build --packages-select imu_complementary_filter
colcon build --packages-select simulation
colcon build --packages-select system_check
```

3. After the compilation, you have to source the ros environment:
```sh
source ~/ros2_ws/install/local_setup.bash
source /opt/ros/humble/setup.bash
```

