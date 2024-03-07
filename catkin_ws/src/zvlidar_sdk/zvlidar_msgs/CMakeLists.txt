cmake_minimum_required(VERSION 3.5)

project(zvlidar_msgs)

if(NOT CMAKE_CXX_STANDARD)
    # set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

#=======================================
# Compile setup (NULL, CATKIN, COLCON)
#=======================================ss
# set(COMPILE_METHOD CATKIN)
set(COMPILE_METHOD COLCON)

# ros
find_package(roscpp  QUIET)
if(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")
  message(=============================================================)
  message("-- ROS Found, Ros Support is turned On!")
  message(=============================================================)
  add_definitions(-DROS_FOUND)
  add_definitions(-DENV_ROS)
else(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")

# ros2
find_package(rclcpp QUIET)
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  message(STATUS "=============================================================")
  message("-- ROS2 Found, Ros2 Support is turned On!")
  message(=============================================================)
  add_definitions(-DROS2_FOUND)
  add_definitions(-DENV_ROS)
else(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  message(=============================================================)
  message("-- ROS2 Not Found, Ros2 Support is turned Off!")
  message(=============================================================)
  message("-- ROS2 Not Found, Ros2 Support is turned Off!")
  message(=============================================================)
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

# ROS
if(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ZvisionLidarPacket_ros1.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ZvisionLidarPacket.msg COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ZvisionLidarScan_ros1.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ZvisionLidarScan.msg COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SFactoryHeartbeatMsg_ros1.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SFactoryHeartbeatMsg.msg COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SHeartbeatMsg_ros1.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SHeartbeatMsg.msg COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SPlusHeartbeatMsg_ros1.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SPlusHeartbeatMsg.msg COPYONLY)
  find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
  add_message_files(
    DIRECTORY msg
    FILES
    ZvisionLidarPacket.msg
    ZvisionLidarScan.msg
    ML30SFactoryHeartbeatMsg.msg
    ML30SHeartbeatMsg.msg
    ML30SPlusHeartbeatMsg.msg
  )
  generate_messages(DEPENDENCIES std_msgs)

  catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
  )
endif(roscpp_FOUND AND ${COMPILE_METHOD} STREQUAL "CATKIN")

# ROS2
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ZvisionLidarPacket_ros2.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ZvisionLidarPacket.msg COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ZvisionLidarScan_ros2.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ZvisionLidarScan.msg COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SFactoryHeartbeatMsg_ros2.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SFactoryHeartbeatMsg.msg COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SHeartbeatMsg_ros2.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SHeartbeatMsg.msg COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SPlusHeartbeatMsg_ros2.msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/ML30SPlusHeartbeatMsg.msg COPYONLY)
  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
          "msg/ZvisionLidarPacket.msg"
          "msg/ZvisionLidarScan.msg"
          "msg/ML30SFactoryHeartbeatMsg.msg"
          "msg/ML30SHeartbeatMsg.msg"
          "msg/ML30SPlusHeartbeatMsg.msg"
          #"msg/MLXSHeartbeatMsg.msg"
          DEPENDENCIES std_msgs
          DEPENDENCIES builtin_interfaces
          ADD_LINTER_TESTS
        )

  if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
