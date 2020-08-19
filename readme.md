#### 1. Prerequisites
(1) Install a ubuntu PC. We suggested Ubuntu 14.04 or Ubuntu 16.04. Please do not use virtual machine.
(2) Install ros full-desktop version. We tried Indigo and Kinect.
(3) Please also install libpcap-dev.

####  2. Install
(1). Copy the whole zvisionlidar ROS driver directory into ROS workspace, i.e "~/catkin_ws/src". Please make sure your dir mane has no special characters, '+', '(', ')' etc.

(2). Check the file attributes:

```
cd ~/catkin_ws/src/zvisionlidar/zvision_lidar_drvier
chmod 777 cfg/*
cd ~/catkin_ws/src/zvisionlidar/zvision_lidar_pointcloud
chmod 777 cfg/*
```

(3). Then to compile the source code and to install it:

```
cd ~/catkin_ws
catkin_make
```
#### 3. Configure PC IP
By default, the ZVISION_LIDAR is configured to **192.168.10.108** as its device IP and **255.255.255.255** as destination IP that it would communicate. The default **LiDAR UDP dst port is 35xx(3500)**.
So you need configure your PC IP as a static one **192.168.10.10**.

#### 4. Run as independent node
We have provide example launch files under zvision_lidar_pointcloud/launch, we can run the launch file to view the point cloud data. For example, if we want to view ML30 real time data:
(1). Open a new terminal and run:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch zvision_lidar_pointcloud zvision_lidar_ML30.launch
```

#### 5. Run as nodelet
We can also run the driver node and cloud node as a nodelet.
Open a new terminal and run:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch zvision_lidar_pointcloud online_sensor_cloud_nodelet.launch
```
Then we can run view the pointcloud via "rviz"

#### 6. About the lidar calibration parameters
Under "**zvision_lidar_pointcloud/data**" directory, you can find the lidar calibration parameters files for the exact sensor. By default the launch file load the files
- zvision_lidar_pointcloud/data/ML30_Default.cal


If you have more than one ZVISIONLIDAR, you can put the data files into "**zvision_lidar_pointcloud/data**" directory.Then you need rewrite the launch file to start your lidar. We have put an example launch file "online_two_sensor_cloud_nodelet.launch" to load two lidars together for reference.

#### 7. Apply translation and rotation to the pointcloud
We could apply translation and rotation to the pointcloud before calculate the x-y-z. This operation is used after a zvision_lidar_node node is running.
(1). Open a new terminal and run:

```
cd ~/catkin_ws
rosrun rqt_reconfigure rqt_reconfigure 
```
(2). Select the pointcloud source node (for example:zvision_lidar_node), and change the x-trans,...,z-trans,...,z-rotation.


**Notes**
In the launch file, we could set the parameter (device_ip and udp_port) to receive the specific lidar data.
In the launch file, we could set the parameter (angle_path) to get the correct angle file.
In the launch file, we could set the parameter (x_tra/y_tra/z_tra) to coordination transformation(x = x_old + x_trans).
In the launch file, we could set the parameter (x_rot/y_rot/z_rot) to coordination transformation(x = rotation (x_old)).
In the launch file, we could set the parameter (timestamp_type) to fill PointCloud2 message timestamp field(0:local ros timestamp, 1:GPS timestamp field in udp packet).











