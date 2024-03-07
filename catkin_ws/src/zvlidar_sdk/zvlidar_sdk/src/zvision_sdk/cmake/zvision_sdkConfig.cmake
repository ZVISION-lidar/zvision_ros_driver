# - Config file for the  package
# It defines the following variables
#  zvision_sdk_INCLUDE_DIRS - include directories for 
#  zvision_sdk_LIBRARIES    - libraries to link against
#  zvision_sdk_FOUND        - found flag


if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")
endif(${ENABLE_TRANSFORM})

# Install
# the sdk include directory is ${ZVISION_SDK_INSTALL_DIR}/include
#                     library directory is ${ZVISION_SDK_INSTALL_DIR}/lib
#                     binary directory is ${ZVISION_SDK_INSTALL_DIR}/bin
get_filename_component(ZVISION_SDK_INSTALL_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(ZVISION_SDK_INCLUDE_DIR ${ZVISION_SDK_INSTALL_DIR}/../include)
set(ZVISION_SDK_LIBRARY_DIR ${ZVISION_SDK_INSTALL_DIR}/../lib)

# Libraries
set(ZVISION_SDK_EXTERNAL_LIBS "pthread")
set(ZVISION_SDK_LIBRARY "zvision_sdk_static")

set(ZVISION_SDK_FOUND true)
