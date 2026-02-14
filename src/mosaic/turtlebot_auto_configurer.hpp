//
// Created by yhkim on 2026. 2. 13.
//

#ifndef MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TURTLEBOT_AUTO_CONFIGURER_HPP
#define MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TURTLEBOT_AUTO_CONFIGURER_HPP

#include <mosaic/auto_configurer/auto_configurer.h>

#include "ctx.hpp"
#include "turtlebot_connector_configurer.hpp"

class TurtlebotAutoConfigurer : public mosaic::auto_configurer::AutoConfigurer {
public:
    TurtlebotAutoConfigurer() = default;

    void BeforeConfigure() override {
        if (ctx_ == nullptr) {
            throw std::runtime_error("Robot Context is not set");
        }
        for (const auto &configurable_connector: configurable_connectors_) {
            if (const auto connector_configurer = std::dynamic_pointer_cast<TurtlebotConnectorConfigurer>(
                configurable_connector)) {
                connector_configurer->SetRobotContext(ctx_);
            }
        }
    }

    void SetRobotContext(const std::shared_ptr<RobotContext> &ctx) {
        ctx_ = ctx;
    }

private:
    std::shared_ptr<RobotContext> ctx_;
};

#endif //MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TURTLEBOT_AUTO_CONFIGURER_HPP
