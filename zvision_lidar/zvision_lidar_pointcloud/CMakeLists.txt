cmake_minimum_required(VERSION 3.5)
project(zvision_lidar_pointcloud)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(zvision_lidar_msgs REQUIRED)
# find_package(zvision_lidar_driver REQUIRED)
# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
        NAMES yaml_cpp.h
        PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
        NAMES YAML_CPP
        PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

include_directories(include/zvision_lidar_pointcloud include/tools ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_subdirectory(src/tools)
add_subdirectory(src/lib)

add_library(convert SHARED
        src/conversions/convert.cc)

ament_target_dependencies(convert
        diagnostic_updater
        Eigen3
        rclcpp
        rclcpp_components
        tf2
        zvision_lidar_msgs
        geometry_msgs
        sensor_msgs
        )
target_link_libraries(convert
        zvision_rawdata
        zvision_lidar_tools
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES})
install(TARGETS convert
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
rclcpp_components_register_nodes(convert
        "zvision_lidar_pointcloud::Convert")



add_executable(zvision_convert_node
        src/conversions/convert_node.cc)
ament_target_dependencies(zvision_convert_node
        rclcpp
        )
target_link_libraries(zvision_convert_node convert ${PCL_LIBRARIES})
install(TARGETS zvision_convert_node
        DESTINATION lib/${PROJECT_NAME}
        )


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config data rviz_cfg
  DESTINATION share/${PROJECT_NAME})

ament_package()













