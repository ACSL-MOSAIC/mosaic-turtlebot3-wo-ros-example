cmake_minimum_required(VERSION 3.14)
project(mosaic_turtlebot3_wo_ros_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    add_compile_definitions(
            _USE_MATH_DEFINES
    )
endif ()

if (POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif ()

set(Boost_NO_BOOST_CMAKE ON)
find_package(dynamixel_sdk QUIET)
find_package(Boost REQUIRED)
find_package(mosaic-core REQUIRED)

# If DynamixelSDK is not found via find_package, configure manually
if (NOT dynamixel_sdk_FOUND)
    message(STATUS "dynamixel_sdk not found via find_package, using manual configuration")
    if (APPLE)
        set(dynamixel_sdk_LIBRARIES reference/DynamixelSDK/c++/build/mac/libdxl_mac_cpp.dylib)
        set(dynamixel_sdk_INCLUDE_DIRS reference/DynamixelSDK/c++/include)

        # Check if library file exists
        if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dynamixel_sdk_LIBRARIES}")
            message(FATAL_ERROR
                    "\n"
                    "===============================================\n"
                    "DynamixelSDK library not found!\n"
                    "Expected location: ${dynamixel_sdk_LIBRARIES}\n"
                    "===============================================\n"
                    "\n"
                    "Please build DynamixelSDK for macOS:\n"
                    "\n"
                    "  1. Clone DynamixelSDK repository:\n"
                    "     git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git reference/DynamixelSDK\n"
                    "\n"
                    "  2. Build for macOS:\n"
                    "     cd reference/DynamixelSDK/c++/build/mac\n"
                    "     make\n"
                    "\n"
                    "  3. Return to project root and try building again:\n"
                    "     cd ../../../../..\n"
                    "     cmake -B build\n"
                    "\n"
                    "===============================================\n"
            )
        endif ()
    else ()
        set(dynamixel_sdk_INCLUDE_DIRS /usr/local/include)
        set(dynamixel_sdk_LIBRARIES dxl_sbc_cpp)
    endif ()
endif ()

add_executable(mosaic_turtlebot3_wo_ros_example
        src/main.cpp
        src/turtlebot3/dynamixel_sdk_wrapper.cpp
        src/hlds/hlds_laser_publisher.cpp
        src/mosaic/laser_scan_sender.cpp
        src/mosaic/teleop_receiver.cpp
)

target_include_directories(mosaic_turtlebot3_wo_ros_example PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
        ${dynamixel_sdk_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(mosaic_turtlebot3_wo_ros_example
        ${dynamixel_sdk_LIBRARIES}
        ${Boost_LIBRARIES}
        mosaic::mosaic-core
)
