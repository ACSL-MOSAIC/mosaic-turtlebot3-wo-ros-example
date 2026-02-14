//
// Created by yhkim on 2026. 2. 13.
//

#ifndef MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_LASER_SCAN_SENDER_HPP
#define MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_LASER_SCAN_SENDER_HPP

#include "../hlds/laser_scan.hpp"
#include "turtlebot_connector_configurer.hpp"
#include <mosaic/auto_configurer/connector/a_dc_handler_configurer.h>
#include <mosaic/handlers/data_channel/data_channel_sendable.h>

struct RobotContext;

class LaserScanSenderConfigurer : public mosaic::auto_configurer::ADCHandlerConfigurer,
                                  public TurtlebotConnectorConfigurer {
public:
    LaserScanSenderConfigurer() = default;

    std::string GetConnectorType() const override {
        return "turtlebot-sender-laser-scan";
    }

    void Configure() override;
};

class LaserScanSender : public mosaic::handlers::DataChannelSendable {
public:
    explicit LaserScanSender(const std::string &channel_name,
                             const std::shared_ptr<RobotContext> &ctx) : DataChannelSendable(channel_name), ctx_(ctx) {
    }

    ~LaserScanSender() override = default;

    void SendLaserScan(const std::shared_ptr<LaserScan> &scan) const;

private:
    std::shared_ptr<RobotContext> ctx_;
};

#endif //MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_LASER_SCAN_SENDER_HPP
