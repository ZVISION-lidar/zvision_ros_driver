cmake_minimum_required(VERSION 3.0.2)
project(zvision_lidar_monitor)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    roslib
    sensor_msgs
    tf
    zvision_lidar_msgs
    nodelet
)

find_package(catkin REQUIRED COMPONENTS
	${${PROJECT_NAME}_CATKIN_DEPS}
    rospy
	std_msgs
    genmsg
    message_generation
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    CATKIN_DEPENDS message_runtime std_msgs
    )

add_subdirectory(src)

install(FILES monitor_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)