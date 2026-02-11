#ifndef MOSAIC_TURTLEBOT3_EX_DEVICES__DEVICES_HPP_
#define MOSAIC_TURTLEBOT3_EX_DEVICES__DEVICES_HPP_

#include <memory>
#include <string>
#include <utility>

#include "control_table.hpp"
#include "dynamixel_sdk_wrapper.hpp"


namespace robotis {
  namespace turtlebot3 {
    extern const ControlTable extern_control_table;

    namespace devices {
      class Devices {
      public:
        explicit Devices(
          std::shared_ptr<DynamixelSDKWrapper> &dxl_sdk_wrapper)
          : dxl_sdk_wrapper_(dxl_sdk_wrapper) {
        }

        virtual void command(const void *request, void *response) = 0;

      protected:
        std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
      };
    } // namespace devices
  } // namespace turtlebot3
} // namespace robotis

#endif  // MOSAIC_TURTLEBOT3_EX_DEVICES__DEVICES_HPP_
