# This file describes the different possibilities for software development: necessary tools, organization tips ...

## Programming STM32 Nucleos boards

Nucleo firmwares are developed with "STM32CubeIDE" (https://www.st.com/en/development-tools/stm32cubeide.html). \
The Nucleo firmwares are available at [F103_Firmware](../nucleoF103/firmware) and [L476_Firmware](../nucleoL476/firmware)

**1. Open a project**

From the IDE, click on "File" -> "Open projects from file system", then select the firmware. 


**2. Compile and run the poject**

You can then compile the firmware : "Project" -> "Build" (or CTRL + B). 
To flash a program and use the debug mode, you have to set the JP5 jumper on U5V (on Nucleos boards), before connecting the board in USB

**3. Debug**

To enter in debug mode : "Run" -> "Debug" (or F11)

**For the NucleoF103 only** : set a breakpoint below the line "power_boostrap();", then run the program until you reach this breakpoint. Power the car (red buttons), then run again. The program is now running.


## Programming Raspberry and Jetson ROS nodes

It is advisable to develop the ROS nodes and compile them locally on your PC, before transferring them to the Raspberry/Jetson. 
The Raspberry takes a long time to compile, so it is best to check that the programs compile correctly on your PC before compiling on 
the Raspberry.

You have to install ROS2 Humble on your PC: https://docs.ros.org/en/humble/Installation.html 

* **If you are running Ubuntu 22.04**, simply follow the installation procedure: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

* **If you are running Windows 10**, you can use an Ubuntu 22.04 virtual machine to install ROS2 Humble
 (recommended if your machine is powerful enough). \
 Otherwise, you can install the ROS2 Humble version for windows 10 : https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html or use docker

* **In other cases or if you prefer**, you can use a docker container :
   - For linux: https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html 
   - For windows : https://docs.docker.com/desktop/install/windows-install/ 

You will then have to install a ros-humble container:

```sh
docker pull osrf/ros:humble-desktop
docker run -it --net=host --name=ros-humble osrf/ros:humble-desktop 
```

   

**Important Tip**: Use git to transfer files from your PC to the Raspberry and the Jetson:

1. Initialize the ros2_ws in git repository, linked to your team git :
```sh
cd ~/ros2_ws
git init
git remote add origin "Link to your team's git repository"
```

2. Do the same thing on your PC

3. You can then simply use the "git push", "git pull" commands to transfer the files

## Use visual studio code with docker

If you work with Visual Studio Code and you use a docker container for ROS, you can use the **VS Code extension "docker remote"**, which allows you to access the directories inside the container : press "F1" then "attach to docker container".
