add_library(zvision_rawdata SHARED
    rawdata.cc
)
ament_target_dependencies(zvision_rawdata
    angles
    geometry_msgs
    rclcpp
    sensor_msgs
    tf2
    tf2_ros
    zvision_lidar_msgs
)
install(TARGETS zvision_rawdata
RUNTIME DESTINATION bin
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
)