add_library(zvision_lidar_data rawdata.cc)
target_link_libraries(zvision_lidar_data
    zvision_lidar_tools
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(zvision_lidar_point convert.cc)
target_link_libraries(zvision_lidar_point
    zvision_lidar_data
    ${catkin_LIBRARIES})


add_library(zvision_lidar_cloud_nodelet cloud_nodelet.cc)
target_link_libraries(zvision_lidar_cloud_nodelet zvision_lidar_point
                      ${catkin_LIBRARIES})

add_subdirectory(tools)
add_executable(zvision_lidar_cloud_node cloud_node.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(zvision_lidar_data ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(zvision_lidar_cloud_node
    zvision_lidar_point
    zvision_lidar_tools
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

#add_subdirectory(tools)

install(TARGETS zvision_lidar_data
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS zvision_lidar_point
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS zvision_lidar_cloud_node
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)

install(TARGETS zvision_lidar_cloud_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



