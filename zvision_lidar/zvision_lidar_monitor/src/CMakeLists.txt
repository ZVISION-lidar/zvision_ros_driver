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

add_executable(demo_monitor_pub demo_monitor_pub.cpp)
add_executable(demo_monitor_sub demo_monitor_sub.cpp)
add_executable(demo_frame_info demo_frame_info.cpp)

add_dependencies(demo_monitor_sub zvision_lidar_msgs_gencpp)
add_dependencies(demo_frame_info zvision_lidar_msgs_gencpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(zvision_lidar_monitor ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(demo_monitor_pub
    zvision_lidar_monitor
	${catkin_LIBRARIES}
)

target_link_libraries(demo_monitor_sub
	${catkin_LIBRARIES}
)

target_link_libraries(demo_frame_info
	${catkin_LIBRARIES}
)

install(TARGETS zvision_lidar_monitor
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS demo_monitor_pub
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)

install(TARGETS demo_monitor_sub
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)

install(TARGETS demo_frame_info
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)

install(TARGETS zvision_lidar_monitor_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)