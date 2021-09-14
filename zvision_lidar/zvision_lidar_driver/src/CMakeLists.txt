add_library(zvision_lidar_input input.cc)
target_link_libraries(zvision_lidar_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES})

add_library(zvision_lidar_driver zvision_driver.cpp)
target_link_libraries(zvision_lidar_driver
  zvision_lidar_input
  ${catkin_LIBRARIES})

# build the nodelet version
add_library(zvision_lidar_driver_nodelet nodelet.cc zvision_driver.cpp)
target_link_libraries(zvision_lidar_driver_nodelet
  zvision_lidar_input
  ${catkin_LIBRARIES}
)

add_executable(zvision_lidar_node zvision_lidar_node.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(zvision_lidar_input ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(zvision_lidar_node
	zvision_lidar_driver
        zvision_lidar_input
	${catkin_LIBRARIES}
	${libpcap_LIBRARIES}
)

install(TARGETS zvision_lidar_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS zvision_lidar_driver
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS zvision_lidar_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS zvision_lidar_driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
















