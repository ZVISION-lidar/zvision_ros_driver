#### 1. Prerequisites
(1) Install a ubuntu PC. We suggested Ubuntu 14.04, Ubuntu 16.04 and Ubuntu 18.04. Please do not use virtual machine.
(2) Install ros full-desktop version. We tried Indigo, Kinect and Melodic.
(3) Please also install libpcap-dev.

####  2. Install
(1). Copy the whole zvisionlidar ROS driver directory into ROS workspace, i.e "~/catkin_ws/src". Please make sure your dir mane has no special characters, '+', '(', ')' etc.

(2). Check the file attributes:

```
cd ~/catkin_ws/src/zvisionlidar/zvision_lidar_driver
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
By default, the ZVISION_LIDAR is configured to **192.168.10.108** as its device IP and **255.255.255.255** as destination IP that it would communicate. The default **LiDAR UDP dst port is 2368**.
So you need configure your PC IP as a static one **192.168.10.10**.

#### 4. Run online lidar
We have provide example launch files under zvision_lidar_pointcloud/launch, we can run the launch file to view the point cloud data. For example, if we want to view ML30/ML30SA1 real time data:
(1). Open a new terminal and run:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch zvision_lidar_pointcloud ML30SA1.launch
```

#### 5. Run offline pcap file
We can also run the driver to view the offline pcap file:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch zvision_lidar_pointcloud ML30SA1_pcap.launch
```
Then we can run view the pointcloud via "rviz"

#### 6. About the lidar calibration parameters
Under "**zvision_lidar_pointcloud/data**" directory, you can find the lidar calibration parameters files for the exact sensor. By default the launch file load the files
- zvision_lidar_pointcloud/data/ML30SA1_Default.cal


If you have more than one ZVISIONLIDAR, you can put the data files into "**zvision_lidar_pointcloud/data**" directory.Then you need rewrite the launch file to start your lidar. We have put an example launch file "ML30SA1_two_sensor.launch" to load two lidars together for reference.

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

#### 8. Launch Parameters 

| Param               | Definition                                                   | Ranges                                                       | Notes                                                        |
| ------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| model               | model of LiDAR                                               | ML30B1 / ML30SA1 / MLX / MLXS                                | "unknown LIDAR model" will be printed in the console if this parameter is set incorrectly. |
| device_ip           | IP of LiDAR                                                  | ---                                                          | if this value does not match the device, there will be no pointcloud message and will get "zvision lidar poll timeout" message in the console. |
| udp_port            | UDP dst port of LiDAR                                        | ---                                                          | if this value does not match the device, there will be no pointcloud message and will get "zvision lidar poll timeout" message in the console. |
| angle_path          | path of calibration file                                     | ---                                                          | If the driver fails to open the file,it will print "Open calibration file error" in the console.<br />if the value is set to "", the driver will get calibration data online (only works in online mode). |
| pcap                | offline pcap file path                                       | ---                                                          | If the program fails to open the file, it will print "Error opening zvision lidar socket dump file" in the console. |
| x_tra,y_tra,z_tra   | extrinsic parameters of LiDAR ( translation )                | ---                                                          | unit: m                                                      |
| x_rot,y_rot,z_rot   | extrinsic parameters of LiDAR ( rotation )                   | ---                                                          | unit: deg                                                    |
| use_lidar_time      | source of PointCloud timestamp                               | true / false                                                 | true: use timestamp from udp datapack <br />false: use timestamp from local system time |
| downsample_type     | type of downsample                                           | downsample_voxel<br />dowensample_line<br />downsample_cfg_file(only for ml30sa1)<br/>none | default: none                                                |
| line_sample         | parameter of line downsample, <br />nly take effect when downsample_type is set to **dowensample_line** | >=1                                                          | Keep the data of first line every N lines in each field of view |
| voxel_leaf_size     | parameter of voxel downsample,<br />only take effect when downsample_type is set to **downsample_voxel** | None-zero value                                              | size of voxel grid (unit: m)                                 |
| use_outlier_removal | outlier_removal switch                                       | true / false                                                 | true: switch on outlier_removal<br />false: switch off outlier_removal |
| outlier_th          | threshold of outliers                                        | positive floating point number                               | *Squared* Euclidean *distance*                               |
| downsample_cfg_path | path of downsample config file                               | only support for lidar ML30SA1                               | If the driver fails to open the file,it will print "Open downsample file failed" in the console.<br />if the value is set to "", the driver will not downsampling the pointcloud. |
| use_lidar_line_id   | lidar line id                                                | only support for lidar ML30SA1 and point type PointXYZIRT    | false: R(ring) expressed lidar fov id<br/>true:  R expressed lidar point line id. |

