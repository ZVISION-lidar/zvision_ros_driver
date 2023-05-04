### 1. Run Online Lidar Heartbeat Monitor


> Start running the demo
>
> 1.1 We have two ways to publish heartbeat information.
> ```tex
> # use nodelet manager
> roslaunch zvision_lidar_monitor ml30sb2_monitor.launch
> ```
> or
> ```tex
> # run node
> rosrun zvision_lidar_monitor demo_monitor_pub
> ```
> 1.2 Running heartbeat subscription node.
> ```tex
> rosrun zvision_lidar_monitor demo_monitor_sub
> ```
>
> If the operation is successful, as shown in the following figure

<img src="res\image-20220805194552271.png" alt="image-20220805194552271" style="zoom:50%;" />



### 2. ML30SB2 Heartbeat Description

Big-endian format, parsing lidar heartbeat packet data

**zvision_lidar_heartbeat header description**

| item     | Description                           |
| -------- | ------------------------------------- |
| stamp    | Heartbeat packet timestamp            |
| data     | Heartbeat data, 138 Bytes for ML30SB2 |
| data_len | Heartbeat data length                 |
|          |                                       |

**zvision_lidar_heartbeat data description** (Udp data segment parsing format description)

| Id   | Item                        | Size (Bytes) | Offset | Description                                                  |
| ---- | --------------------------- | ------------ | ------ | ------------------------------------------------------------ |
| 1    | tag                         | 12           | 0      | ZVSHEARTBEAT                                                 |
| 2    | version                     | 2            | 12     | heartbeat version                                            |
| 3    | sn                          | 18           | 14     | serial number                                                |
| 4    | fpga_version                | 4            | 32     | fpga version                                                 |
| 5    | fpga_file_size              | 4            | 36     | fpga file version                                            |
| 6    | env_version                 | 4            | 40     | env version                                                  |
| 7    | env_file_size               | 4            | 44     | env file size                                                |
| 8    | embadded_version            | 4            | 48     | embadded version                                             |
| 9    | embadded_file_size          | 4            | 52     | embadded file size                                           |
| 10   | sys_diag_status             | 1            | 56     | system diagnostic status                                     |
| 11   | hardware_diag_status        | 4            | 57     | hardware diagnostic status                                   |
| 12   | ptp_sync_status             | 1            | 61     | for bit0: 1 lock, 0 unlock<br/>bit4: 1 dirty, 0 clear        |
| 13   | reserved                    | 2            | 62     | -                                                            |
| 14   | ip                          | 4            | 64     | lidar ip address                                             |
| 15   | port                        | 4            | 68     | lidar port number                                            |
| 16   | time_sync_mode              | 1            | 72     | PTP: 0x01, GPS:0x02                                          |
| 17   | dst_ip                      | 4            | 73     | lidar destination ip address                                 |
| 18   | retro_switch                | 1            | 77     | retro algorithm switch<br/>Off: 0x00, On: 0x01               |
| 19   | net_mask                    | 4            | 78     | lidar subnet mask                                            |
| 20   | cfg_mac_addr                | 6            | 82     | lidar config mac address                                     |
| 21   | frame_sync_offset           | 4            | 88     | frame offset value(uint 5ns)                                 |
| 22   | echo_mode                   | 1            | 92     | EchoSingleFirst: 0x01<br/>EchoSingleStrongest: 0x02<br/>EchoSingleLast: 0x04<br/>EchoDoubleFirstStrongest: 0x03<br/>EchoDoubleFirstLast: 0x05<br/>EchoDoubleStrongestLast: 0x06 |
| 23   | frame_sync_switch           | 1            | 93     | frame sync switch<br/>Off: 0x00, On: 0x01                    |
| 24   | retro_intensity_percentage  | 2            | 94     | retro algorithm parameters:<br/>intensity: byte 94<br/>percentage: byte 95 |
| 25   | angle_send_switch           | 1            | 96     | automatically send angle file switch<br/>Off: 0x00, On: 0x01 |
| 26   | downsample_mode             | 1            | 97     | downsampling mode<br>none: 0x00<br/>1/2: 0x01<br/>1/4: 0x02  |
| 27   | dirty_check_thres_set_reset | 4            | 98     | dirty check threshold<br/>set value: bytes 98,99<br/>reset value: bytes 10,101 |
| 28   | dirty_switch                | 1            | 102    | dirty switch<br/>Off: 0x00, On: 0x01                         |
| 29   | dirty_fresh_switch          | 1            | 103    | dirty fresh  switch<br/>Off: 0x00, On: 0x01                  |
| 30   | dirty_detect_cycle          | 2            | 104    | dirty detect cycle                                           |
| 31   | diag_switch                 | 1            | 106    | diagnose switch<br/>Off: 0x00, On: 0x01                      |
| 32   | diag_inner_thres            | 2            | 107    | dirty Internal detect lossing points threshold               |
| 33   | diag_outer_thres            | 2            | 109    | dirty external detect lossing points threshold               |
| 34   | point_loss_thres            | 2            | 111    | dirty detect lossing points threshold                        |
| 35   | diag_sys_sw                 | 1            | 113    | system diagnostic switch (Table 1.1)                         |
| 36   | diag_hardware_sw            | 4            | 114    | hardware diagnostic switch (Table 1.2)                       |
| 37   | dhcp_switch                 | 1            | 118    | dhcp switch<br/>Off: 0x00, On: 0x01                          |
| 38   | gateway                     | 4            | 119    | gateway address                                              |
| 39   | del_point_switch            | 1            | 123    | delete point algorithm switch<br/>Off: 0x00, On: 0x01        |
| 40   | adhesion_switch             | 1            | 124    | adhesion algorithm switch<br/>Off: 0x00, On: 0x01            |
| 41   | para_cfg_switch             | 1            | 125    | configuration parameter switch<br/>Off: 0x00, On: 0x01       |
| 42   | unified_cfg_version         | 3            | 126    | Configuration parameter function switch                      |
| 43   | reserved                    | 3            | 129    | -                                                            |
| 44   | mac_addr                    | 6            | 132    | lidar mac address (currently in use)                         |

> Table 1.1
>
> | Id   | Description<br/>switch: 0 Off, 1 On<br/>state:    0 normal, 1 error |
> | ---- | ------------------------------------------------------------ |
> | bit7 | -                                                            |
> | bit6 | -                                                            |
> | bit5 | Point Cloud loss fault                                       |
> | bit4 | Supply voltage fault                                         |
> | bit3 | Software Failure                                             |
> | bit2 | (package loss)  Hardware Failure                             |
> | bit1 | Over  temperature                                            |
> | bit0 | Low  temperature Start                                       |
>
> Table 1.2
>
> | Id          | Description <br/>switch: 0 Off, 1 On<br/>state:    0 normal, 1 error, |
> | ----------- | ------------------------------------------------------------ |
> | bit31:bit16 | -                                                            |
> | bit15       | LD12V voltage                                                |
> | bit14       | RB5.0V voltage                                               |
> | bit13       | RB3.3V voltage                                               |
> | bit12       | 3.3V voltage                                                 |
> | bit11       | 1.8V voltage                                                 |
> | bit10       | 5.5V voltage                                                 |
> | bit9        | 5.5V power                                                   |
> | bit8        | PAC19xx state                                                |
> | bit7        | APD bias voltage                                             |
> | bit6        | Mems high voltage                                            |
> | bit5        | Mems drive voltage                                           |
> | bit4        | I2C communication                                            |
> | bit3        | Down fov start ghost state                                   |
> | bit2        | Up fov start ghost state                                     |
> | bit1        | ADC1 CLK                                                     |
> | bit0        | ADC0 CLK                                                     |
>
> 

