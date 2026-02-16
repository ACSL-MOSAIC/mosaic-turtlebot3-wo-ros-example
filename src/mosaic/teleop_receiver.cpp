//
// Created by yhkim on 2026. 2. 13.
//

#include "teleop_receiver.hpp"
#include "ctx.hpp"

void TeleopReceiverConfigurer::Configure() {
    handler_ = std::make_shared<TeleopReceiver>(connector_config_->label, ctx_);
}

void TeleopReceiver::HandleData(const Json::Value &data) {
    if (!data.isMember("linear_x") || !data.isMember("angular_z")) {
        MOSAIC_LOG_ERROR("Invalid teleop message: missing 'linear_x' or 'angular_z' fields.");
        return;
    }

    float linear_x = data["linear_x"].asFloat();
    float angular_z = data["angular_z"].asFloat();

    ctx_->teleop->linear_x = linear_x;
    ctx_->teleop->angular_z = angular_z;
    ctx_->teleop_changed->store(true);

    MOSAIC_LOG_INFO("Teleop command received: linear_x = {}, angular_z = {}", linear_x, angular_z);
}
