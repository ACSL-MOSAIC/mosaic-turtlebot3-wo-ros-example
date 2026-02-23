//
// Created by yhkim on 2026. 2. 13.
//

#include "teleop_receiver.hpp"
#include "ctx.hpp"

void TeleopReceiverConfigurer::Configure() {
    handler_ = std::make_shared<TeleopReceiver>(connector_config_->label, ctx_);
}

void TeleopReceiver::HandleData(const Json::Value &data) {
    if (!data.isMember("linear") || !data.isMember("angular")) {
        MOSAIC_LOG_ERROR("Invalid teleop message: missing 'linear' or 'angular' fields.");
        return;
    }

    const Json::Value &linear = data["linear"];
    const Json::Value &angular = data["angular"];
    if (!linear.isMember("x") || !angular.isMember("z")) {
        MOSAIC_LOG_ERROR("Invalid teleop message: missing 'linear.x' or 'angular.z' fields.");
        return;
    }

    if (!ctx_->teleop) {
        ctx_->teleop = std::make_shared<Teleop>();
    }

    ctx_->teleop->linear.x = linear["x"].asFloat();
    ctx_->teleop->angular.z = angular["z"].asFloat();

    float linear_x = linear["x"].asFloat();
    float angular_z = angular["z"].asFloat();

    ctx_->teleop->linear.x = linear_x;
    ctx_->teleop->angular.z = angular_z;
    ctx_->teleop_changed->store(true);

    MOSAIC_LOG_INFO("Teleop command received: linear_x = {}, angular_z = {}", linear_x, angular_z);
}
