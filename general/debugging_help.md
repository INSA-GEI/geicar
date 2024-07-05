
# This file describes possible problems encountered and how to try to solve them

## I. Errors on the system_check report (when launching raspberry nodes)

### 1. Communications status errors
A FAILED communication status indicates that the raspberry cannot communicate with the corresponding board.

**Nucleos FAILED** (possible reasons) :
* the nucleo board has crashed 
* CAN communication is not working

**Jetson FAILED**\
It indicates that the raspberry can't communicate with the ROS environment of the Jetson. 
Possible reasons are : 
* the ROS nodes of the Jetson have not been started 
* the ROS_ID of the Jetson is not the same as the Raspberry's ROS_ID (see ~/.bashrc, line "export ROS_DOMAIN_ID=XX")
* the ethernet communication does not work 

### 2. Sensors status errors

* **Ultrasonics sensors**

**If all ultrasonic sensors indicate an "out of range" error message, just restart the car.**

1. "Out of range" error : the corresponding sensor returns an inconsistent value. Check the correct functioning of the sensor
2. "No data received" error : if the communication with the nucleoF103 works, then this problem is unknown

* **GPS**
1. "No fix" error : this indicates that the **GPS is working**, but **has not yet found a position**
2. "No data received" error : if the communication with the nucleoL476 works, then the GPS is not connected or not working
3. "Autonomous GNSS fix" warning : this indicates that the **GPS is working**, and that it has found a position **without receiving a correction from the base**
4. "Differential GNSS fix" warning : this indicates that the **GPS is working**, and that it has found a position **without receiving a correction from the base**

* **IMU**
"No data received" error : if the communication with the nucleoL476 works, then the IMU is not connected or not working

* **LIDAR**
"No data received" error : if the communication with the Jetson works, then the LIDAR is not connected or not working

* **CAMERA**
"No data received" error : if the communication with the Jetson works, then the CAMERA is not connected or not working
