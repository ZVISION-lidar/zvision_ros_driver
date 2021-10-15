cmake_minimum_required(VERSION 3.5)
project(zvision_lidar)
set(CMAKE_CXX_STANDARD 14)
find_package(ament_cmake REQUIRED)
ament_package()
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)