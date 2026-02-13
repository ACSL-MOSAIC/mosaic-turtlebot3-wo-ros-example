//
// Created by yhkim on 2026. 2. 13.
//

#ifndef MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_CTX_HPP
#define MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_CTX_HPP
#include <memory>
#include <boost/asio.hpp>

#include "hlds/lfcd_laser.hpp"
#include "mosaic/laser_scan_sender.hpp"
#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

typedef struct {
    float separation;
    float radius;
} Wheels;

typedef struct {
    float profile_acceleration_constant;
    float profile_acceleration;
} Motors;

struct Teleop {
    float linear_x;
    float angular_z;
};

struct RobotContext {
    // DXL related
    std::shared_ptr<robotis::turtlebot3::DynamixelSDKWrapper> dxl_sdk_wrapper;
    std::shared_ptr<Motors> motors;
    std::shared_ptr<Wheels> wheels;
    // LD-01 related
    std::shared_ptr<boost::asio::io_context> io_context;
    std::shared_ptr<hls_lfcd_lds::LFCDLaser> laser;
    // LaserScan related
    std::shared_ptr<LaserScanSender> laser_scan_sender;
    // Teleop related
    std::shared_ptr<Teleop> teleop;
    std::shared_ptr<std::atomic_bool> teleop_changed;
};

#endif //MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_CTX_HPP
