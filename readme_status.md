# BSD lidar status by ROI point
please refer the `ML30SA1_pcap_status.launch` and `ML30SA1_status.launch` launch file

## Launch Parameters 

| Param | Definition | Ranges  | Default value |
| ------ | ------- | ------- | ------- |
| use_pointcloud_status_diagnose | switch of pointcloud status diagnose |     true / false    | false |
| statistic_frame_num       |   statistic frame num      |   (0,6000)  | 100 |
| roi_sample_lines  |     number of ROI sampled lines   |    (0,80)     | 10 |
| roi_interval  |     ROI pointcloud sample interval per row    |    (0,40)     | 10 |
| min_roi_pointnum  |     roi min pointnum    |     (0,5000)    | 60 |
| z_height  |    roi average z height (m)     |    (-2, 0)     | -0.58 |
| roi_z_diff_threshold  |    roi z difference threshold     |     (0,1)   |  0.1 |
| error_rate_threshold  |    error rate to the statistic frame num       |    (0,1)     | 0.5 |

## ROI mode

select the bottom pointcloud of the 1 , 2 field of view as ROI poingclouds,
use the `roi_sample_lines` and `roi_interval` adjust the sampled pointclouds

`o` means one fov pointclouds, `x` means sampled roi points
```
                        |--------- 80 ----------|         _____
                        oooooooooooooooooo  ... o           |
                                   .                        |
                                   .                        |
                                   .                        |
                        oooooooooooooooooo  ... o           |
                        oooooooooooooooooo  ... o           |
____                    oooooooooooooooooo  ... o           |
 |                      xoooooooooxooooooo  ... o           80
 |                      xoooooooooxooooooo  ... o           |
 |                      xoooooooooxooooooo  ... o           |
                                    .                       |
roi_sample_lines                    .                       |
 |                      xoooooooooxooooooo  ... o           |
 |                                                        __|__
————                    |───┬────| 
                            └-> roi_interval          
```
## dynamic reconfigure param

use the rqt_reconfigure to dynamic reconfigure param 
```
rosrun rqt_reconfigure rqt_reconfigure
```

## output msg
the status will be publish in the `zvisionLidarInformation.msg`
if lidar status error, the value `is_pc_status_error` will be True
