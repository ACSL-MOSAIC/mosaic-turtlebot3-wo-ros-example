//
// Created by yhkim on 2026. 2. 13.
//

#ifndef MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_LASER_SCAN_HPP
#define MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_LASER_SCAN_HPP

#include <vector>

struct LaserScan {
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

#endif //MOSAIC_TURTLEBOT3_WO_ROS_EXAMPLE_LASER_SCAN_HPP
