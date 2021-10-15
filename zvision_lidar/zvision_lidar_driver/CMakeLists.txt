cmake_minimum_required(VERSION 3.5)
project(zvision_lidar_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(zvision_lidar_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
if(PCL_FOUND)
	include_directories(${PCL_INCLUDE_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	link_directories(${PCL_LIBRARY_DIRS})
endif()
# find_package(pcl_ros REQUIRED)
# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include/zvision_lidar_driver)

# compile the driver and input library
add_subdirectory(src/lib)

# build the driver node
add_library(zvision_lidar_driver SHARED src/driver/zvision_driver.cpp)
ament_target_dependencies(zvision_lidar_driver
		diagnostic_msgs
		diagnostic_updater
		rclcpp
		rclcpp_components
		tf2_ros
		zvision_lidar_msgs
		)
target_link_libraries(zvision_lidar_driver zvision_input)
#
add_executable(zvision_lidar_node src/driver/zvision_lidar_node.cpp)
ament_target_dependencies(zvision_lidar_node
		rclcpp
		)
target_link_libraries(zvision_lidar_node zvision_lidar_driver)

# install runtime and library files
install(TARGETS zvision_lidar_driver
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
		)
install(TARGETS zvision_lidar_node
		DESTINATION lib/${PROJECT_NAME}
		)

rclcpp_components_register_nodes(zvision_lidar_driver
		"zvision_lidar_driver::zvisionLidarDriver")

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})


ament_package()









