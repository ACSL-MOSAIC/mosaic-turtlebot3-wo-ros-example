//
// Created by yhkim on 2026. 2. 13.
//

#ifndef MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TURTLEBOT_CONNECTOR_CONFIGURER_HPP
#define MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TURTLEBOT_CONNECTOR_CONFIGURER_HPP
#include <memory>

struct RobotContext;

class TurtlebotConnectorConfigurer {
public:
    TurtlebotConnectorConfigurer() = default;

    void SetRobotContext(const std::shared_ptr<RobotContext> &ctx) {
        ctx_ = ctx;
    }

protected:
    std::shared_ptr<RobotContext> ctx_;
};

#endif //MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_TURTLEBOT_CONNECTOR_CONFIGURER_HPP
