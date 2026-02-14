#include <iostream>
#include <memory>
#include <atomic>
#include <random>
#include <boost/asio.hpp>

#include <mosaic/auto_configurer/connector/connector_resolver.h>

#include "ctx.hpp"
#include "mosaic/turtlebot_auto_configurer.hpp"
#include "mosaic/laser_scan_sender.hpp"
#include "mosaic/teleop_receiver.hpp"
#include "hlds/laser_scan.hpp"
#include "hlds/lfcd_laser.hpp"
#include "turtlebot3/dynamixel_sdk_wrapper.hpp"
#include "turtlebot3/control_table.hpp"

using namespace robotis::turtlebot3;

// Global flag to signal threads to stop
std::atomic should_stop(false);

std::shared_ptr<DynamixelSDKWrapper> init_dynamixel_sdk_wrapper(const std::string &usb_port);

void check_device_status(const std::shared_ptr<DynamixelSDKWrapper> &dxl_sdk_wrapper_);

void main_loop(const std::shared_ptr<RobotContext> &ctx);

std::thread main_loop_thread(const std::shared_ptr<RobotContext> &ctx, std::chrono::milliseconds timeout) {
    std::cout << "Start Main Loop every " << 1000.0f / timeout.count() << " Hz" << std::endl;
    return std::thread([ctx, timeout]() {
        while (!should_stop) {
            main_loop(ctx);
            std::this_thread::sleep_for(timeout);
        }
        std::cout << "Main loop thread stopped" << std::endl;
    });
}

void heartbeat(const std::shared_ptr<RobotContext> &ctx);

std::thread heartbeat_thread(const std::shared_ptr<RobotContext> &ctx, std::chrono::milliseconds timeout) {
    std::cout << "Start Heartbeat every " << 1000.0f / timeout.count() << " Hz" << std::endl;
    return std::thread([ctx, timeout]() {
        while (!should_stop) {
            heartbeat(ctx);
            std::this_thread::sleep_for(timeout);
        }
        std::cout << "Heartbeat thread stopped" << std::endl;
    });
}

void cmd_vel(const std::shared_ptr<RobotContext> &ctx);

std::thread cmd_vel_thread(const std::shared_ptr<RobotContext> &ctx, std::chrono::milliseconds timeout) {
    std::cout << "Start CmdVel every " << 1000.0f / timeout.count() << " Hz" << std::endl;
    return std::thread([ctx, timeout]() {
        while (!should_stop) {
            cmd_vel(ctx);
            std::this_thread::sleep_for(timeout);
        }
        std::cout << "CmdVel thread stopped" << std::endl;
    });
}

void laser(const std::shared_ptr<RobotContext> &ctx);

std::thread laser_thread(const std::shared_ptr<RobotContext> &ctx, std::chrono::milliseconds timeout) {
    std::cout << "Start Laser every " << 1000.0f / timeout.count() << " Hz" << std::endl;
    return std::thread([ctx, timeout]() {
        while (!should_stop) {
            laser(ctx);
            std::this_thread::sleep_for(timeout);
        }
        std::cout << "Laser thread stopped" << std::endl;
    });
}

int main() {
    const auto dxl_usb_port = "/dev/ttyACM0";
    const auto hlds_usb_port = "/dev/ttyUSB0";
    const auto mosaic_config_path = "/app/mosaic_config.yaml";

    // Initialize turtlebot dxl sdk
    const auto dxl_sdk_wrapper = init_dynamixel_sdk_wrapper(dxl_usb_port);
    check_device_status(dxl_sdk_wrapper);

    const auto motors_ = std::make_shared<Motors>(Motors{214.577f, 0.0f});
    const auto wheels_ = std::make_shared<Wheels>(Wheels{0.160f, 0.033f});

    // Initialize laser components
    const auto hlds_io_context = std::make_shared<boost::asio::io_context>();
    const auto laser_ = std::make_shared<hls_lfcd_lds::LFCDLaser>(hlds_usb_port, 230400, *hlds_io_context);

    // Create robot context
    const auto context = std::make_shared<RobotContext>(RobotContext{
        dxl_sdk_wrapper,
        motors_,
        wheels_,
        hlds_io_context,
        laser_,
        nullptr,
        nullptr,
        std::make_shared<std::atomic_bool>(false)
    });

    // Initialize MOSAIC
    mosaic::auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<TeleopReceiverConfigurer>();
    mosaic::auto_configurer::ConnectorResolver::GetInstance().RegisterConfigurableConnector<
        LaserScanSenderConfigurer>();

    const auto auto_configurer = std::make_shared<TurtlebotAutoConfigurer>();
    auto_configurer->SetRobotContext(context);
    auto_configurer->AutoConfigure(mosaic_config_path);

    auto main_loop_thread_ = main_loop_thread(context, std::chrono::milliseconds(100));
    auto heartbeat_thread_ = heartbeat_thread(context, std::chrono::milliseconds(5000));
    auto cmd_vel_thread_ = cmd_vel_thread(context, std::chrono::milliseconds(3000));
    auto laser_thread_ = laser_thread(context, std::chrono::milliseconds(100));

    std::cout << "Wait until Keyboard Interrupt" << std::endl;
    while (true) {
        try {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } catch (const std::exception &e) {
            std::cout << e.what() << std::endl;
            break;
        }
    }

    // Signal all threads to stop
    should_stop = true;
    std::cout << "Stopping threads..." << std::endl;

    try {
        auto_configurer->GetMosaicConnector()->ShuttingDown();
    } catch (...) {
    }

    if (main_loop_thread_.joinable()) {
        main_loop_thread_.join();
    }

    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }

    if (cmd_vel_thread_.joinable()) {
        cmd_vel_thread_.join();
    }

    if (laser_thread_.joinable()) {
        laser_thread_.join();
    }

    std::cout << "All threads stopped" << std::endl;

    return 0;
}

std::shared_ptr<DynamixelSDKWrapper> init_dynamixel_sdk_wrapper(const std::string &usb_port) {
    DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

    std::cout << "Init DynamixelSDKWrapper" << std::endl;

    const auto dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

    dxl_sdk_wrapper_->init_read_memory(
        extern_control_table.millis.addr,
        (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) +
        extern_control_table.profile_acceleration_right.length
    );

    return dxl_sdk_wrapper_;
}

void check_device_status(const std::shared_ptr<DynamixelSDKWrapper> &dxl_sdk_wrapper_) {
    if (dxl_sdk_wrapper_->is_connected_to_device()) {
        std::string sdk_msg;
        uint8_t reset = 1;

        dxl_sdk_wrapper_->set_data_to_device(
            extern_control_table.imu_re_calibration.addr,
            extern_control_table.imu_re_calibration.length,
            &reset,
            &sdk_msg);

        std::cout << "Start Calibration of Gyro" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << "Calibration End" << std::endl;
    } else {
        std::cout << "Failed connection with Devices" << std::endl;
        return;
    }

    const auto device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
        extern_control_table.device_status.addr,
        extern_control_table.device_status.length);

    if (constexpr int8_t NOT_CONNECTED_MOTOR = -1; device_status == NOT_CONNECTED_MOTOR) {
        std::cout << "Please check your Dynamixels and Power" << std::endl;
    }
}

void main_loop(const std::shared_ptr<RobotContext> &ctx) {
    // TODO: Add main loop logic using ctx->dxl_sdk_wrapper, ctx->motors, ctx->wheels
}

void heartbeat(const std::shared_ptr<RobotContext> &ctx) {
    static uint8_t count = 0;
    std::string msg;

    ctx->dxl_sdk_wrapper->set_data_to_device(
        extern_control_table.heartbeat.addr,
        extern_control_table.heartbeat.length,
        &count,
        &msg);

    std::cout << "hearbeat count : " << static_cast<int>(count) << ", msg : " << msg << std::endl;

    count++;
}

void cmd_vel(const std::shared_ptr<RobotContext> &ctx) {
    if (!ctx->teleop_changed->load()) {
        return;
    }
    if (ctx->teleop == nullptr) {
        ctx->teleop_changed->store(false);
        return;
    }

    ctx->teleop_changed->store(false);

    const auto linear_x = ctx->teleop->linear_x;
    const auto angular_z = ctx->teleop->angular_z;

    constexpr auto linear_x_max = 2.2;
    constexpr auto angular_z_max = 28.4;

    std::string sdk_msg;

    union Data {
        int32_t dword[6];
        uint8_t byte[4 * 6];
    } data{};

    data.dword[0] = static_cast<int32_t>(linear_x * linear_x_max);
    data.dword[1] = 0;
    data.dword[2] = 0;
    data.dword[3] = 0;
    data.dword[4] = 0;
    data.dword[5] = static_cast<int32_t>(angular_z * angular_z_max);

    const uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
    const uint16_t addr_length =
            (extern_control_table.cmd_velocity_angular_z.addr -
             extern_control_table.cmd_velocity_linear_x.addr) +
            extern_control_table.cmd_velocity_angular_z.length;

    uint8_t *p_data = &data.byte[0];

    ctx->dxl_sdk_wrapper->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

    std::cout << "cmd_vel - lin_vel: " << linear_x * linear_x_max << " ang_vel: " << angular_z * angular_z_max
            << " msg : " << sdk_msg.c_str() << std::endl;
}

void laser(const std::shared_ptr<RobotContext> &ctx) {
    if (ctx->laser == nullptr || ctx->laser_scan_sender) return;
    const auto scan = std::make_shared<LaserScan>();
    ctx->laser->poll(scan);
    if (scan->ranges.empty()) return;
    ctx->laser_scan_sender->SendLaserScan(scan);
}
