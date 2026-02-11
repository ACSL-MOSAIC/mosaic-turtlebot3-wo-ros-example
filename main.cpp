#include <iostream>
#include <memory>
#include <atomic>

#include "dynamixel_sdk_wrapper.hpp"
#include "control_table.hpp"

using namespace robotis::turtlebot3;

// Global flag to signal threads to stop
std::atomic<bool> should_stop(false);

typedef struct {
    float separation;
    float radius;
} Wheels;

typedef struct {
    float profile_acceleration_constant;
    float profile_acceleration;
} Motors;

struct RobotContext {
    std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper;
    std::shared_ptr<Motors> motors;
    std::shared_ptr<Wheels> wheels;
};

std::shared_ptr<DynamixelSDKWrapper> init_dynamixel_sdk_wrapper(const std::string &usb_port);

void check_device_status(const std::shared_ptr<DynamixelSDKWrapper> &dxl_sdk_wrapper_);

void main_loop(const std::shared_ptr<RobotContext> &ctx);

std::thread main_loop_thread(const std::shared_ptr<RobotContext> &ctx, std::chrono::milliseconds timeout) {
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
    return std::thread([ctx, timeout]() {
        while (!should_stop) {
            cmd_vel(ctx);
            std::this_thread::sleep_for(timeout);
        }
        std::cout << "CmdVel thread stopped" << std::endl;
    });
}

int main() {
    const auto usb_port = "/dev/ttyUSB0";

    const auto dxl_sdk_wrapper = init_dynamixel_sdk_wrapper(usb_port);
    check_device_status(dxl_sdk_wrapper);

    auto motors_ = std::make_shared<Motors>(Motors{214.577f, 0.0f});
    auto wheels_ = std::make_shared<Wheels>(Wheels{0.160f, 0.033f});

    // Create robot context
    auto context = std::make_shared<RobotContext>(RobotContext{
        dxl_sdk_wrapper,
        motors_,
        wheels_
    });

    std::cout << "Start Main Loop" << std::endl;
    auto main_loop_thread_ = main_loop_thread(context, std::chrono::milliseconds(100));

    std::cout << "Start Heartbeat" << std::endl;
    auto heartbeat_thread_ = heartbeat_thread(context, std::chrono::milliseconds(100));

    std::cout << "Start CmdVel" << std::endl;
    auto cmd_vel_thread_ = cmd_vel_thread(context, std::chrono::milliseconds(100));

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

    if (main_loop_thread_.joinable()) {
        main_loop_thread_.join();
    }

    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }

    if (cmd_vel_thread_.joinable()) {
        cmd_vel_thread_.join();
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

    const int8_t NOT_CONNECTED_MOTOR = -1;

    int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
        extern_control_table.device_status.addr,
        extern_control_table.device_status.length);

    switch (device_status) {
        case NOT_CONNECTED_MOTOR:
            std::cout << "Please double check your Dynamixels and Power" << std::endl;
            break;

        default:
            break;
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
}

void cmd_vel(const std::shared_ptr<RobotContext> &ctx) {
    // TODO: Add cmd_vel logic using ctx->dxl_sdk_wrapper, ctx->motors, ctx->wheels
}
