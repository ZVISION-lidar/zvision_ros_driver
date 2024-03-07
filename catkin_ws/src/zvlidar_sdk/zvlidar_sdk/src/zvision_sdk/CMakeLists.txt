cmake_minimum_required(VERSION 3.0)

project(zvision_sdk VERSION 1.0.1)

add_compile_options(-std=c++11 -std=c++14)

if (CMAKE_CROSSCOMPILING)
	set(THREADS_PTHREAD_ARG
		"PLEASE_FILL_OUT-FAILED_TO_RUN"
		CACHE STRING "Result from TRY_RUN" FORCE)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

#========================
# Define
#========================
set(SDK_LIBRARY_NAME ${PROJECT_NAME}_static)

#========================
# Path To Find SDK
#========================
# build
set(SDK_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/sdk/include)
set(SDK_LIBRARY_DIRS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
# install
set(SDK_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME})
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/include)
set(INSTALL_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/lib)
set(INSTALL_BINARY_DIR ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/bin)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/cmake)

if(WIN32)
else()
  list(APPEND EXTERNAL_LIBS pthread)
endif(WIN32)

#========================
#  Item
#========================
add_subdirectory(sdk)
add_subdirectory(sample/pointcloud)
add_subdirectory(sample/lidar_config)

#========================
#  Cmake
#========================
set(SDK_CMAKE_ROOT ${CMAKE_CURRENT_LIST_DIR}/cmake)

configure_file (
  ${CMAKE_CURRENT_LIST_DIR}/sdk/macro/version.h.in
  ${CMAKE_CURRENT_LIST_DIR}/sdk/include/version.h @ONLY
)

configure_file(
  ${SDK_CMAKE_ROOT}/zvision_sdkConfig.cmake.in
  ${SDK_CMAKE_ROOT}/zvision_sdkConfig.cmake @ONLY
)

configure_file(
  ${SDK_CMAKE_ROOT}/zvision_sdkConfigVersion.cmake.in
  ${SDK_CMAKE_ROOT}/zvision_sdkConfigVersion.cmake @ONLY
)

if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  set(zvision_sdk_DIR ${SDK_CMAKE_ROOT} PARENT_SCOPE)
endif()

#========================
#  Install
#========================
install(FILES ${SDK_CMAKE_ROOT}/zvision_sdkConfig.cmake
                         ${SDK_CMAKE_ROOT}/zvision_sdkConfigVersion.cmake
    DESTINATION ${INSTALL_CMAKE_DIR})
install(DIRECTORY sdk/include/
    DESTINATION ${INSTALL_INCLUDE_DIR})