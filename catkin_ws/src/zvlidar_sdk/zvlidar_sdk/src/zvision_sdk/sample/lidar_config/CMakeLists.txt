cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidarconfig_sample)
add_executable(${DEMO_NAME} main.cpp
               convert.hpp
              )

add_executable(${DEMO_NAME}_ml30sp_b1 sample_ml30sp_b1.cpp
               convert.hpp
              )

add_executable(${DEMO_NAME}_ml30sp_b1_ep1 sample_ml30sp_b1_ep1.cpp
               convert.hpp
              )

add_executable(${DEMO_NAME}_ml30sp_b1_IS1 sample_ml30sp_b1_IS1.cpp
               convert.hpp
              )

target_link_libraries(${DEMO_NAME}
        ${SDK_LIBRARY_NAME}
        -lpthread
        )

target_link_libraries(${DEMO_NAME}_ml30sp_b1
        ${SDK_LIBRARY_NAME}
        -lpthread
        )

target_link_libraries(${DEMO_NAME}_ml30sp_b1_ep1
        ${SDK_LIBRARY_NAME}
        -lpthread
        )
target_link_libraries(${DEMO_NAME}_ml30sp_b1_IS1
        ${SDK_LIBRARY_NAME}
        -lpthread
        )
