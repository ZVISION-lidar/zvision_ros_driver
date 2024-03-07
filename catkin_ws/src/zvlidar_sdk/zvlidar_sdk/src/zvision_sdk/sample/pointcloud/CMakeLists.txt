cmake_minimum_required(VERSION 3.0)
find_package(PCL 1.7 COMPONENTS common io visualization)

set(DEMO_NAME pointcloud_sample)
add_executable(${DEMO_NAME} main.cpp)

set(DEMO_LIBS ${SDK_LIBRARY_NAME})
message(STATUS "##########")
if(PCL_FOUND)
    add_definitions(-DUSING_PCL_VISUALIZATION)
    target_include_directories(${DEMO_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
    list(APPEND DEMO_LIBS ${PCL_LIBRARIES})
    message(STATUS "PCL library found")
else(PCL_FOUND)
    message(WARNING "PCL library not found")
endif(PCL_FOUND)

#message(STATUS "PCL library found " ${DEMO_LIBS})
target_link_libraries(${DEMO_NAME}
    ${DEMO_LIBS}
    -lpthread)
