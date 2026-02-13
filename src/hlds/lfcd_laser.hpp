#ifndef HLS_LFCD_LDS_DRIVER__LFCD_LASER_HPP_
#define HLS_LFCD_LDS_DRIVER__LFCD_LASER_HPP_


#include <string>

#include "laser_scan.hpp"
#include <boost/asio.hpp>
#include <boost/array.hpp>

namespace hls_lfcd_lds {
  class LFCDLaser {
  public:
    uint16_t rpms; ///< @brief RPMS derived from the rpm bytes in an LFCD packet
    /**
    * @brief Constructs a new LFCDLaser attached to the given serial port
    * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
    * @param baud_rate The baud rate to open the serial port at.
    * @param io Boost ASIO IO Service to use when creating the serial port object
    */
    LFCDLaser(const std::string &port, uint32_t baud_rate, boost::asio::io_context &io);

    /**
    * @brief Default destructor
    */
    ~LFCDLaser();

    /**
    * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
    * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
    */
    void poll(const std::shared_ptr<LaserScan> &scan);

    /**
    * @brief Close the driver down and prevent the polling loop from advancing
    */
    void close() { shutting_down_ = true; }

  private:
    std::string port_;
    uint32_t baud_rate_;
    bool shutting_down_;
    boost::asio::serial_port serial_;
    uint16_t motor_speed_;
  };
} // namespace hls_lfcd_lds
#endif  // HLS_LFCD_LDS_DRIVER__LFCD_LASER_HPP_
