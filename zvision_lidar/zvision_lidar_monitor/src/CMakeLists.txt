add_library(zvision_lidar_monitor monitor.cc)
target_link_libraries(zvision_lidar_monitor
 ${catkin_LIBRARIES}
)

# add zvision_lidar_monitor_nodelet nodelet
add_library(zvision_lidar_monitor_nodelet nodelet.cc monitor.cc)
target_link_libraries(zvision_lidar_monitor_nodelet
  zvision_lidar_monitor
  ${catkin_LIBRARIES}
)

add_executable(demo_monitor demo_monitor.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(zvision_lidar_monitor ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(demo_monitor
    zvision_lidar_monitor
	${catkin_LIBRARIES}
)

install(TARGETS zvision_lidar_monitor
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS demo_monitor
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)

install(TARGETS zvision_lidar_monitor_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)