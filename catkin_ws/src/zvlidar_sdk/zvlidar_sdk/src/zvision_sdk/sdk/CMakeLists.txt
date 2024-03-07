cmake_minimum_required(VERSION 3.0)

#set(ZVISION_SDK_LIBRARY ${PROJECT_NAME}_static)
add_library(${SDK_LIBRARY_NAME} STATIC "")

#set(ZVISION_SDK_MAJOR_VERSION "0")
#set(ZVISION_SDK_MINOR_VERSION "1")
#set(ZVISION_SDK_PATCH_VERSION "13")
#set(ZVISION_SDK_VERSION_STRING "${ZVISION_SDK_MAJOR_VERSION}.${ZVISION_SDK_MINOR_VERSION}.${ZVISION_SDK_PATCH_VERSION}")

target_include_directories(${SDK_LIBRARY_NAME}
        PUBLIC
        include/common
        include
        PRIVATE
        include/commu
        include/protocol
        include/json)

set_target_properties(${SDK_LIBRARY_NAME} PROPERTIES PUBLIC_HEADER "include/lidar_tools.h;include/point_cloud.h;include/common/define.h;include/common/print.h;include/loguru.hpp")

target_compile_options(${SDK_LIBRARY_NAME}
        #PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:AppleClang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>)

target_sources(${SDK_LIBRARY_NAME}
        PRIVATE
        src/lidar_tools.cpp
        src/packet.cpp
        src/packet_source.cpp
        src/point_cloud.cpp
        src/print.cpp
        src/client.cpp
        src/define.cpp
        src/loguru.cpp)

install(TARGETS ${SDK_LIBRARY_NAME}
        ARCHIVE DESTINATION ${INSTALL_LIBRARY_DIR}
        LIBRARY DESTINATION ${INSTALL_LIBRARY_DIR})
