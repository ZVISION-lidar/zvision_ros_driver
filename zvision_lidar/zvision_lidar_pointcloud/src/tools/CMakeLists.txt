add_library(zvision_lidar_tools SHARED tcp_client.cc tools.cc)
include_directories(./)
ament_target_dependencies(zvision_lidar_tools
        rclcpp
)

install(TARGETS zvision_lidar_tools
RUNTIME DESTINATION bin
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
)
# install(TARGETS zvision_lidar_tools
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )
