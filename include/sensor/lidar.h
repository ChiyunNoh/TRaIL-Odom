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

#ifndef TRAIL_LIDAR_H
#define TRAIL_LIDAR_H

#include "memory"
#include "util/utils.hpp"
#include "ctraj/utils/macros.hpp"
#include "ctraj/utils/utils.hpp"

namespace trail {

    struct LidarTarget {
    public:
        using Ptr = std::shared_ptr<LidarTarget>;

    private:
        // the timestamp of this frame
        std::pair<double, double> _timestamp;

        PointCloudXYZIN::Ptr _scan;

    public:
        explicit LidarTarget(std::pair<double, double> timestamp = std::pair<double,double>{INVALID_TIME_STAMP, INVALID_TIME_STAMP},
                             PointCloudXYZIN::Ptr scan = std::make_shared<PointCloudXYZIN>());

        static LidarTarget::Ptr Create(
            const std::pair<double, double> &timestamp = std::pair<double,double>{INVALID_TIME_STAMP, INVALID_TIME_STAMP},
            const PointCloudXYZIN::Ptr &scan = std::make_shared<PointCloudXYZIN>());

        // access
        [[nodiscard]] PointCloudXYZIN::Ptr GetScan() const;

        [[nodiscard]] std::pair<double, double> GetTimestamp() const;

        void SetTimestamp(std::pair<double, double> timestamp);

        friend std::ostream &operator<<(std::ostream &os, const LidarTarget &frame);

    };

}

#endif 
