add_library(zvision_input SHARED input.cc)
include_directories(../../include/zvision_lidar_driver)
ament_target_dependencies(zvision_input
    rclcpp
    zvision_lidar_msgs
)
target_link_libraries(zvision_input
    ${libpcap_LIBRARIES}
)

install(TARGETS zvision_input
    LIBRARY DESTINATION lib
)