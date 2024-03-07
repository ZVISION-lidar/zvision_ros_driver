cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(zvlidar_sdk)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
# set(COMPILE_METHOD CATKIN)	# ROS1
set(COMPILE_METHOD COLCON)	# ROS2

#========================
# Project details / setup
#========================
set(PROJECT_NAME zvlidar_sdk)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
set(CMAKE_BUILD_TYPE RELEASE)
add_definitions(-O3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++11 -std=c++14 -std=c++17)
# add_compile_options(-std=c++17)
add_compile_options(-Wall -g)

if(${COMPILE_METHOD} STREQUAL "ORIGINAL")
    # Output path defination
    set(CMAKE_INSTALL_PREFIX /home/uu/zvision)
    set(CMAKE_DEBUG_POSTFIX "_d")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/lib)
    set(CMAKE_LIBARAY_OUTPUT_DIRECTORY         ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/lib)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY       ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/bin)
    set(CMAKE_EXECUTABLE_OUTPUT_PATH            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/bin)
endif(${COMPILE_METHOD} STREQUAL "ORIGINAL")

#========================
# Dependencies Setup
#========================
#ROS#
find_package(roscpp QUIET)
if(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")
  message(=============================================================)
  message("-- ROS Found, Ros Support is turned On!")
  message(=============================================================)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  add_definitions(-DROS_FOUND)
  add_definitions(-DENV_ROS)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)

  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

else(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")

#ROS2#
find_package(rclcpp QUIET)
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  message(=============================================================)
  message("-- ROS2 Found, Ros2 Support is turned On!")
  message(=============================================================)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  add_definitions(-DROS2_FOUND)
  add_definitions(-DENV_ROS)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD 11)

  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(zvlidar_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  
  find_package(ament_cmake REQUIRED)
  #find_package(example_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(pluginlib REQUIRED)

  # by su
  # if(rclcpp_components_FOUND)
  #   message(~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~)
  #   message("rclcpp_components found. Version: ${rclcpp_components_VERSION}")
  #   message(~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~)
  #   message("Include directories: ${rclcpp_components_INCLUDE_DIRS}")
  #   message(~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~)
  #   message("Libraries: ${rclcpp_components_LIBRARIES}")
  #   message(~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~)
  # else()
  #   message(FATAL_ERROR "rclcpp_components not found. Please make sure it is installed.")
  # endif()


  find_package(rcutils)
  # find_package(rmw REQUIRED)
  #find_package(std_msgs REQUIRED)
  include_directories(${rclcpp_INCLUDE_DIRS})

else(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  message(=============================================================)
  message("-- ROS2 Not Found, Ros2 Support is turned Off!")
  message(=============================================================)
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

# Find yaml, PCL #
find_package(yaml-cpp REQUIRED)
find_package(PCL QUIET REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Catkin#
if(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")
  add_definitions(-DRUN_IN_ROS_WORKSPACE)
  find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      sensor_msgs
      roslib
      pcl_ros
      pcl_conversions
      zvlidar_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      roscpp
      sensor_msgs
      roslib
      pcl_conversions
      zvlidar_msgs
  )
  include_directories( ${catkin_INCLUDE_DIRS})
endif(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
add_subdirectory(src/zvision_sdk)
find_package(zvision_sdk REQUIRED)
include_directories(${ZVISION_SDK_INCLUDE_DIR})
link_directories(${ZVISION_SDK_LIBRARY_DIR})
#========================
# Build Setup
#========================

add_library(filter_pointcloud STATIC src/plugin/filter_pointcloud.cpp
  src/plugin/dbscan.cpp
  src/plugin/blooming.cpp
)

target_link_libraries(filter_pointcloud
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${ZVISION_SDK_LIBRARY}
        ${ZVISION_SDK_EXTERNAL_LIBS}
)

# add manager library  SHARED
add_library(processor_manager STATIC src/manager/processor_manager.cpp
    src/processor/processor.cpp
    src/processor/diagnostic_processor.cpp
    src/util/translater.cpp
)


target_link_libraries(processor_manager
        filter_pointcloud
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${ZVISION_SDK_LIBRARY}
        ${ZVISION_SDK_EXTERNAL_LIBS}
)


add_executable(zvlidar_node
    node/zvlidar_node.cpp
    src/manager/processor_manager.cpp
    src/processor/processor.cpp
    src/processor/diagnostic_processor.cpp
    src/plugin/filter_pointcloud.cpp
    src/plugin/dbscan.cpp
    src/plugin/blooming.cpp
    src/util/translater.cpp
    src/util/env_def.h
)

target_link_libraries(zvlidar_node
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${ZVISION_SDK_LIBRARY}
        ${ZVISION_SDK_EXTERNAL_LIBS}
)

#Ros#
if(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")
    target_link_libraries(zvlidar_node ${ROS_LIBS})

    # add_library(zvlidar_nodelet ${PROJECT_SOURCE_DIR}/node/zvlidar_nodelet.cc)
    # target_link_libraries(zvlidar_nodelet
    #     ${PCL_LIBRARIES}
    #     ${YAML_CPP_LIBRARIES}
    #     ${ZVISION_SDK_LIBRARY}
    #     ${ZVISION_SDK_EXTERNAL_LIBS}
    #     ${ROS_LIBS}
    # )

    install(TARGETS zvlidar_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

    # install(TARGETS zvlidar_nodelet  
    #         LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    # 
    # install(FILES zvlidar_nodelet.xml
    #     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)
    install(DIRECTORY config/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
            PATTERN ".svn" EXCLUDE)
endif(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")

#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  ament_target_dependencies(zvlidar_node rclcpp sensor_msgs std_msgs zvlidar_msgs rclcpp_components pluginlib) 
  ament_target_dependencies(processor_manager rclcpp sensor_msgs std_msgs zvlidar_msgs rclcpp_components pluginlib) 

  # add_library(zvlidar_nodelet SHARED ${PROJECT_SOURCE_DIR}/node/zvlidar_nodelet.cc)
  # ament_target_dependencies(zvlidar_nodelet
  #       rclcpp sensor_msgs std_msgs zvlidar_msgs
  #       # ${PCL_LIBRARIES}
  #       # ${YAML_CPP_LIBRARIES}
  #       # ${ZVISION_SDK_LIBRARY}
  #       # ${ZVISION_SDK_EXTERNAL_LIBS}
  #       # ${ROS_LIBS}
  # )

  # target_link_libraries(zvlidar_nodelet processor_manager)
  # rclcpp_components_register_node(zvlidar_nodelet
  #   PLUGIN "zvlidar_sdk::ZVLidarNodelet"
  #   EXECUTABLE test_zvlidar_nodelet)

  install(TARGETS processor_manager
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # install(TARGETS zvlidar_nodelet
  #   ARCHIVE DESTINATION lib
  #   LIBRARY DESTINATION lib
  #   RUNTIME DESTINATION bin
  # )

  install(TARGETS
  zvlidar_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
  )
  ament_package()
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
