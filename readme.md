# zvision_ros_driver for ROS2-foxy with outlier removal
## 1. Prerequisites
(1) Ubuntu 20.04  with ros2-foxy desktop-full version.   
(2) libpcap-dev.  
```bash
sudo apt-get install -y  libpcap-dev
```

##  2. Install
(1). Copy the whole zvisionlidar ROS driver directory into ROS2 workspace, i.e "~/ros2_ws/src". Please make sure your dir mane has no special characters, '+', '(', ')' etc. 
```bash
cd ~/ros2_ws/src
git clone -b ros2-foxy https://github.com/ZVISION-lidar/zvision_ros_driver
``` 
(2). Clone the diagnostics repo
```bash
 git clone -b foxy https://github.com/ros/diagnostics.git 
 ```
(3). Compile with colcon:
```bash
cd ~/ros2_ws
colcon build --symlink-install 
```
## 3. Network configuration
By default, the ZVISION_LIDAR is configured to **192.168.10.108** as its device IP and **255.255.255.255** as destination IP that it would communicate. The default **LiDAR UDP dst port is 2368**.
So you need configure your PC IP as a static one like **192.168.10.10**.

## 4. Get LiDAR data
Take ML30S for example:
### (1) Online 
* Modify config files  
    * >zvision_lidar_driver/config/ML30S-zvision_lidar_driver_node-params.yaml  
    
        * modify the *model* , *device_ip* , *udp_port* to ensure they are the same as your lidar configuration.  
        * modify the frame_id if you need.
        * keep *pcap: ""*  to get online data (enable online mode).
        * *read_once* and *read_fast* are not used under online mode. 

    * >zvision_lidar_pointcloud/config/ML30S-zvision_lidar_convert_node-params.yaml
        ```yaml
        device_ip: 192.168.10.108 # modify to your device configuration
        angle_path: "" # keep "" to get calibration data online
        model: ML30SA1 # modify to your device model, keep ML30SA1 if you are using ML30S serise.
        downsample_type: "" # voxel/line
        line_sample: 2 
        voxel_leaf_size: 0.2f # voxel grid filter parameter.
        use_lidar_time: false # false for ros timestamp, true for timestamp from udp packages.
        x_tra: 0.0 # for pointcloud transformation.
        y_tra: 0.0
        z_tra: 0.0
        x_rot: 0.0
        y_rot: 0.0
        z_rot: 0.0
        ```

* launch   
    ```bash
    cd ~/catkin_ws
    colcon build --symlink-install #install launch and config files into share.
    source install/setup.bash   
    # or source install/setup.zsh if you are using zsh.
    ros2 launch zvision_lidar default_online_zvision_all_nodes_ml30s_launch.py 
    ```
### (2) Offline

You can also get point cloud data from captured pcap data. 
The operation is almost the same as online mode.
* Modify config files  
    * modify the *model* , *device_ip* , *udp_port* to ensure they are the same as your lidar configuration.  
    * modify the frame_id if you need.
    * specify the path of pcap file in *ML30S-zvision_lidar_driver_node-params.yaml* and the angle_path in *ML30S-zvision_lidar_convert_node-params.yaml*.
    * modify the value of *read_once* and *read_fast* based on your need.
* launch   
    ```bash
    cd ~/catkin_ws
    colcon build --symlink-install #install launch and config files into share.
    source install/setup.bash   
    # or source install/setup.zsh if you are using zsh.
    ros2 launch zvision_lidar default_offline_zvision_all_nodes_ml30s_launch.py 
    ```

### (3) Use outlier removal
Modify convert_params to:

```python
convert_params["use_outlier_removal"] = True
convert_params["outlier_th"] = 0.25
# outlier_th: threshold of outliers(the square of the Euclidean distance)
```










