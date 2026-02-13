// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/asio.hpp>

#include "lfcd_laser.hpp"

namespace hls_lfcd_lds {
  LFCDLaser::LFCDLaser(const std::string &port, uint32_t baud_rate, boost::asio::io_context &io)
    : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_) {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

    boost::asio::write(serial_, boost::asio::buffer("b", 1)); // start motor
  }

  LFCDLaser::~LFCDLaser() {
    boost::asio::write(serial_, boost::asio::buffer("e", 1)); // stop motor
  }

  void LFCDLaser::poll(const std::shared_ptr<LaserScan> &scan) {
    uint8_t start_count = 0;
    bool got_scan = false;
    boost::array<uint8_t, 2520> raw_bytes;
    uint8_t good_sets = 0;
    uint32_t motor_speed = 0;
    rpms = 0;
    int index;

    while (!shutting_down_ && !got_scan) {
      // Wait until first data sync of frame: 0xFA, 0xA0
      boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));

      if (start_count == 0) {
        if (raw_bytes[start_count] == 0xFA) {
          start_count = 1;
        }
      } else if (start_count == 1) {
        if (raw_bytes[start_count] == 0xA0) {
          start_count = 0;

          // Now that entire start sequence has been found, read in the rest of the message
          got_scan = true;

          boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 2518));

          scan->angle_increment = 2.0 * M_PI / 360.0;
          scan->angle_min = 0.0;
          scan->angle_max = 2.0 * M_PI - scan->angle_increment;
          scan->range_min = 0.12;
          scan->range_max = 3.5;
          scan->ranges.resize(360);
          scan->intensities.resize(360);

          for (uint16_t i = 0; i < raw_bytes.size(); i = i + 42) {
            if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 42)) {
              good_sets++;
              motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];

              for (uint16_t j = i + 4; j < i + 40; j = j + 6) {
                index = 6 * (i / 42) + (j - 4 - i) / 6;

                // Four bytes per reading
                uint8_t byte0 = raw_bytes[j];
                uint8_t byte1 = raw_bytes[j + 1];
                uint8_t byte2 = raw_bytes[j + 2];
                uint8_t byte3 = raw_bytes[j + 3];

                uint16_t intensity = (byte1 << 8) + byte0;
                uint16_t range = (byte3 << 8) + byte2;

                scan->ranges[359 - index] = range / 1000.0;
                scan->intensities[359 - index] = intensity;
              }
            }
          }

          rpms = motor_speed / good_sets / 10;
          scan->time_increment = static_cast<float>(1.0 / (rpms * 6));
          scan->scan_time = scan->time_increment * 360;
        } else {
          start_count = 0;
        }
      }
    }
  }
} // namespace hls_lfcd_lds
