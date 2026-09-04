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

#include "sensor/lidar.h"

#include <utility>

namespace trail {

    LidarTarget::LidarTarget(std::pair<double, double> timestamp, PointCloudXYZIN::Ptr scan)
        : _timestamp(std::move(timestamp)),
        _scan(std::move(scan)) {}

    LidarTarget::Ptr LidarTarget::Create(const std::pair<double, double> &timestamp, const PointCloudXYZIN::Ptr &scan) {
        return std::make_shared<LidarTarget>(timestamp, scan);
    }

    PointCloudXYZIN::Ptr LidarTarget::GetScan() const { return this->_scan; }

    std::pair<double, double> LidarTarget::GetTimestamp() const { return _timestamp; }

    std::ostream &operator<<(std::ostream &os, const LidarTarget &frame) {
        os << "size: " << frame._scan->size() << ", width: " << frame._scan->width
        << ", height: " << frame._scan->height << ", timestamp: " << frame._timestamp.first;
        return os;
    }

    void LidarTarget::SetTimestamp(std::pair<double, double> timestamp) { _timestamp = timestamp; }

}
