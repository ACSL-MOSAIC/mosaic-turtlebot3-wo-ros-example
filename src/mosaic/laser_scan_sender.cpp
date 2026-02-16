//
// Created by yhkim on 2026. 2. 13.
//

#include "laser_scan_sender.hpp"

#include "ctx.hpp"

void LaserScanSenderConfigurer::Configure() {
    const auto laser_scan_sender = std::make_shared<LaserScanSender>(connector_config_->label, ctx_);
    ctx_->laser_scan_sender = laser_scan_sender;
    handler_ = laser_scan_sender;
}

void LaserScanSender::SendLaserScan(const std::shared_ptr<LaserScan> &scan) const {
    if (!Sendable()) {
        return;
    }

    MOSAIC_LOG_INFO("Send laser scan");

    Json::Value laser_scan_json;
    laser_scan_json["angle_min"] = scan->angle_min;
    laser_scan_json["angle_max"] = scan->angle_max;
    laser_scan_json["angle_increment"] = scan->angle_increment;
    laser_scan_json["time_increment"] = scan->time_increment;
    laser_scan_json["scan_time"] = scan->scan_time;
    laser_scan_json["range_min"] = scan->range_min;
    laser_scan_json["range_max"] = scan->range_max;

    laser_scan_json["ranges"] = Json::Value(Json::arrayValue);
    for (const auto &range: scan->ranges) {
        laser_scan_json["ranges"].append(range);
    }
    laser_scan_json["intensities"] = Json::Value(Json::arrayValue);
    for (const auto &intensity: scan->intensities) {
        laser_scan_json["intensities"].append(intensity);
    }

    SendJson(laser_scan_json);
}
