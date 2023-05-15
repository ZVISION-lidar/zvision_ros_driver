cmake_minimum_required(VERSION 2.8.3)
project(zvision_lidar_pointcloud)

add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)#RelWithDebInfo
set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    zvision_lidar_driver
    zvision_lidar_msgs
    dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} pcl_conversions)
find_package(Boost COMPONENTS signals)
find_package(PkgConfig REQUIRED)
include_directories(${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

generate_dynamic_reconfigure_options(cfg/CloudNode.cfg)
catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    CATKIN_DEPENDS message_runtime std_msgs
    )

add_subdirectory(src)

install(FILES cloud_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
