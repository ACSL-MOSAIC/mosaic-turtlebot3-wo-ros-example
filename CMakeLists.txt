cmake_minimum_required(VERSION 3.14)
project(mosaic_turtlebot3_wo_ros_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(dynamixel_sdk REQUIRED)

add_executable(mosaic_turtlebot3_wo_ros_example main.cpp)

target_include_directories(mosaic_turtlebot3_wo_ros_example PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${dynamixel_sdk_INCLUDE_DIRS}
)
target_link_libraries(mosaic_turtlebot3_wo_ros_example
        ${dynamixel_sdk_LIBRARIES}
)
