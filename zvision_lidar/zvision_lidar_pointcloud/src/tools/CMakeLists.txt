add_library(zvision_lidar_tools tcp_client.cc tools.cc)
include_directories(./)

install(TARGETS zvision_lidar_tools
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
