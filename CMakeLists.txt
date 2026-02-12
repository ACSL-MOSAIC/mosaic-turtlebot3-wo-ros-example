cmake_minimum_required(VERSION 3.14)
project(mosaic_turtlebot3_wo_ros_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(dynamixel_sdk)

# DynamixelSDK가 find_package로 찾아지지 않으면 수동으로 설정
if (NOT dynamixel_sdk_FOUND)
    message(STATUS "dynamixel_sdk not found via find_package, using manual configuration")
    set(dynamixel_sdk_INCLUDE_DIRS /usr/local/include)
    set(dynamixel_sdk_LIBRARIES dxl_sbc_cpp)
endif ()

add_executable(mosaic_turtlebot3_wo_ros_example
        src/main.cpp
        src/dynamixel_sdk_wrapper.cpp
)

target_include_directories(mosaic_turtlebot3_wo_ros_example PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
        ${dynamixel_sdk_INCLUDE_DIRS}
)
target_link_libraries(mosaic_turtlebot3_wo_ros_example
        ${dynamixel_sdk_LIBRARIES}
)
