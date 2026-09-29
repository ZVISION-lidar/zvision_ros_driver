#### 1. Prerequisites
(1) Install a ubuntu PC. We suggested Ubuntu 14.04, Ubuntu 16.04 and Ubuntu 18.04. Please do not use virtual machine.
(2) Install ros full-desktop version. We tried  Melodic and foxy(ROS2).
(3) Please also install libpcap-dev.

####  2. Install
(1). Copy the whole zvlidar_sdk ROS driver directory into ROS workspace, i.e "~/catkin_ws/src". Please make sure your dir mane has no special characters, '+', '(', ')' etc.

(2). Then to compile the source code and to install it:

```
cd ~/catkin_ws
catkin_make
```
#### 3. Configure PC IP
By default, the ZVISION_LIDAR is configured to **192.168.10.108** as its device IP and **255.255.255.255** as destination IP that it would communicate. The default **LiDAR UDP dst port is 2368**.
So you need configure your PC IP as a static one **192.168.10.10**.

#### 4. Run online lidar or  offline pcap file
We have provide example launch files under zvlidar_sdk/launch, we can run the launch file to view the point cloud data. For example, if we want to view ML30/ML30SA1 real time data:
(1). Open a new terminal and run:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch zvlidar_sdk run.launch
```

Then we can run view the pointcloud via "rviz"

#### 6. About the lidar calibration parameters
Under "**zvlidar_sdk/data**" directory, you can find the lidar calibration parameters files for the exact sensor. 









