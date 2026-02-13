//
// Created by yhkim on 2026. 2. 13.
//

#ifndef MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TELEOP_RECEIVER_HPP
#define MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TELEOP_RECEIVER_HPP

#include <mosaic/auto_configurer/connector/a_dc_handler_configurer.h>
#include <mosaic/handlers/data_channel/data_channel_receivable.h>

#include "turtlebot_connector_configurer.hpp"

class TeleopReceiverConfigurer : public mosaic::auto_configurer::ADCHandlerConfigurer,
                                 public TurtlebotConnectorConfigurer {
public:
    TeleopReceiverConfigurer() = default;

    std::string GetConnectorType() const override {
        return "turtlebot-receiver-teleop";
    }

    void Configure() override;
};

class TeleopReceiver : public mosaic::handlers::DataChannelJsonReceivable {
public:
    explicit TeleopReceiver(const std::string &channel_name,
                            const std::shared_ptr<RobotContext> &ctx) : DataChannelJsonReceivable(channel_name),
                                                                        ctx_(ctx) {
    }

    ~TeleopReceiver() override = default;

    void HandleData(const Json::Value &data) override;

private:
    std::shared_ptr<RobotContext> ctx_;
};

#endif //MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TELEOP_RECEIVER_HPP
