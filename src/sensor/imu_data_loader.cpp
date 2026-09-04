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

#include "sensor/imu_data_loader.h"
#include "util/enum_cast.hpp"
#include "core/status.h"
#include "spdlog/fmt/fmt.h"

namespace trail {

    IMUDataUnpacker::Ptr IMUDataUnpacker::Create() {
        return std::make_shared<IMUDataUnpacker>();
    }

    IMUFrame::Ptr IMUDataUnpacker::Unpack(const sensor_msgs::msg::Imu::ConstPtr &msg, double acc_scale) {

        auto acce = Eigen::Vector3d(
                acc_scale * msg->linear_acceleration.x,
                acc_scale * msg->linear_acceleration.y,
                acc_scale * msg->linear_acceleration.z
        );

        auto gyro = Eigen::Vector3d(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z
        );

        auto orientation = Eigen::Quaternion<double>(
                msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z
        );

        return IMUFrame::Create(rclcpp::Time(msg->header.stamp).seconds(), gyro, acce, orientation); 
    }

}
