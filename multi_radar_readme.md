使用ros目录下zvlidar_sdk/config/multi_config.yaml

多雷达参考这个文件。

```yaml

lidar:
  ###############################################################################
  - base:
      lidar_type: nz1_a2            # Lidar type: ez6_b2/nz1_a2
      lidar_ip: 192.168.10.108      # Lidar ip address
      lidar_port: 2368                 # Lidar destination port
      online: true                    # true:  play online lidar
                                                 # false: play pcap file
      ## optional
      multicast_en: false              # Enable to join multicast
      multicast_ip: 224.0.0.1          # multicast ip address
      # pcap_path:  /home/z/Downloads/6.0.22算法关.pcap   # Play offline pointcloud data from pcap_path
      angle_path: 
      #angle_path:  /home/book/ros_test/src/zvlidar_sdk/src/lidar_sdk/doc/nz1_a2_angle_comp.csv 
      pcap_rate: 10                    # Frame rate for play of offline pcap point cloud data. Unit:hz
      ## Pointcloud debug
      x_tra: 0
      y_tra: 0
      z_tra: 0
      x_rot: 0
      y_rot: 0
      z_rot: 0
      ## ros
    ros:
      frame_id: zvlidar
      topic_tag_name: zvlidar       # if topic_tag_name is zvlidar_ml30s
                                        # if pub_pointcloud_xyzi is true, the topic name is zvlidar_ml30s_points_xyzi
                                        #  . pub_pointcloud_xyzirt . . . . , . . . . . . . . . . . . . . . . zvlidar_ml30s_points_xyzirt
                                        #  . pub_pointcloud_xyzrgba . . , . . . . . . . . . . . . . . . . zvlidar_ml30s_points_xyzrgba
      use_lidar_time: true              # true: use lidar timestamp from first udp packet data.  false: use system timestamp from first udp packet that captured.
      use_lidar_line_id: false          # for PointCloudXYZIRT, if true R:point line id, else R:fov id
      ## Pointcloud Topic Switch
      pub_lidar_packet: false           # lidar frame raw data
      pub_pointcloud_xyzi: false        # PointCloudXYZI
      pub_pointcloud_xyzirt: true       # PointCloudXYZIRT
      pub_pointcloud_xyzrgba: false     # PointCloudXYZRGBA
      pub_imu_msg: false                #imu messages
      
  - base:
      lidar_type: nz1_a2            # Lidar type: ez6_b2/nz1_a2
      lidar_ip: 192.168.10.109      # 修改后的 Lidar IP 地址
      lidar_port: 2368              # Lidar destination port
      online: true                  # true: play online lidar
                                    # false: play pcap file
      ## optional
      multicast_en: false           # Enable to join multicast
      multicast_ip: 224.0.0.1       # multicast ip address
      # pcap_path: /home/z/Downloads/6.0.22算法关.pcap   # Play offline pointcloud data from pcap_path
      angle_path: 
      #angle_path: /home/book/ros_test/src/zvlidar_sdk/src/lidar_sdk/doc/nz1_a2_angle_comp.csv 
      pcap_rate: 10                 # Frame rate for play of offline pcap point cloud data. Unit:hz
      ## Pointcloud debug
      x_tra: 0
      y_tra: 0
      z_tra: 0
      x_rot: 0
      y_rot: 0
      z_rot: 0
      ## ros
    ros:
      frame_id: zvlidar_rear
      topic_tag_name: zvlidar_rear       # if topic_tag_name is zvlidar_ml30s
                                    # if pub_pointcloud_xyzi is true, the topic name is zvlidar_ml30s_points_xyzi
                                    #  . pub_pointcloud_xyzirt . . . . , . . . . . . . . . . . . . . . . zvlidar_ml30s_points_xyzirt
                                    #  . pub_pointcloud_xyzrgba . . , . . . . . . . . . . . . . . . . zvlidar_ml30s_points_xyzrgba
      use_lidar_time: true          # true: use lidar timestamp from first udp packet data. false: use system timestamp
      use_lidar_line_id: false      # for PointCloudXYZIRT, if true R:point line id, else R:fov id
      ## Pointcloud Topic Switch
      pub_lidar_packet: false       # lidar frame raw data
      pub_pointcloud_xyzi: false    # PointCloudXYZI
      pub_pointcloud_xyzirt: true   # PointCloudXYZIRT
      pub_pointcloud_xyzrgba: false # PointCloudXYZRGBA
      pub_imu_msg: false            # imu messages

```

修改第二雷达的ip，frame_id和topic_tag_name。重新编译后，运行。

### 以上图的yaml为例子

#### 单雷达显示点云

启动rviz后，点击add，添加第二个点云话题/zvlidar_rear_points_xyzirt，frame_id设置成zvlidar_rear，就会有第二个雷达的点云，frame_id设置成zvlidar，就是第一个雷达的点云。

#### 多雷达同时显示点云

运行命令

```sh
ros2 run tf2_ros static_transform_publisher -2.0 0 0 0 0 0 zvlidar zvlidar_rear
```

 ROS 2发布一个**静态坐标变换（static transform）**，从 `zvlidar` 坐标系到 `zvlidar_rear` 坐标系的变换。
(-2.0 0 0 0 0 0)根据实际情况标定。

- `-2.0 0 0`：表示在 `x` 方向上偏移 -2.0 米（即 `zvlidar_rear` 在 `zvlidar` 坐标系后方 2 米），y 和 z 方向无偏移。
- `0 0 0`：表示没有旋转（欧拉角 yaw、pitch、roll 均为 0 弧度）。
- `zvlidar`：父坐标系（parent frame）。
- `zvlidar_rear`：子坐标系（child frame）。

之后frame_id设置成zvlidar，就会有两个雷达的点云了。
