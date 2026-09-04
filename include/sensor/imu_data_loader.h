// TRaIL-Odom: Tightly Coupled Continuous Time Radar-IMU-LiDAR Odometry
//             with Adaptive Doppler Weighting
// SPDX-License-Identifier: MIT
//
// Copyright (c) 2024 School of Geodesy and Geomatics, Wuhan University
//   Based on: River: A Tightly-Coupled Radar-Inertial Velocity Estimator
//   Upstream: https://github.com/Unsigned-Long/River
//   Original author: Shuolong Chen
//
// Copyright (c) 2026 Chiyun Noh, Turcan Tuna, William Talbot, Marco Hutter,
//   Laurent Kneip, and Ayoung Kim
//
// See LICENSE for the full MIT License text.

#ifndef TRAIL_IMU_DATA_LOADER_H
#define TRAIL_IMU_DATA_LOADER_H

#include "sensor_msgs/msg/imu.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor/imu.h"
#include <rclcpp/rclcpp.hpp>



namespace trail {
    enum class IMUMsgType {
        SENSOR_IMU,
        SBG_IMU
    };

    class IMUDataUnpacker {
    public:
        using Ptr = std::shared_ptr<IMUDataUnpacker>;

    public:
        explicit IMUDataUnpacker() = default;

        static IMUDataUnpacker::Ptr Create();

        static IMUFrame::Ptr Unpack(const sensor_msgs::msg::Imu::ConstPtr &msg, double acc_scale = 1.0);
    };

}


#endif //TRAIL_IMU_DATA_LOADER_H
