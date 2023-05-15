# LIDAR frame information.
bool is_ptp                  # true: ptp, false: gps
bool lock_status             # ptp/gps lock status, true: lock, false: unlock
time stamp                   # first udp packet timestamp in one frame pointcloud
float32 apd_bias             # lidar apd bias
bool is_pc_status_error      # lidar status by ROI ground pointcloud
