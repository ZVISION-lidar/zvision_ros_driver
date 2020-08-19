add_library(zvision_lidar_data rawdata.cc)
target_link_libraries(zvision_lidar_data
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(zvision_lidar_point convert.cc)
target_link_libraries(zvision_lidar_point
    zvision_lidar_data
    ${catkin_LIBRARIES})


add_library(zvision_lidar_cloud_nodelet cloud_nodelet.cc)
target_link_libraries(zvision_lidar_cloud_nodelet zvision_lidar_point
                      ${catkin_LIBRARIES})

add_executable(zvision_lidar_cloud_node cloud_node.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(zvision_lidar_data ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(zvision_lidar_cloud_node
    zvision_lidar_point
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})
