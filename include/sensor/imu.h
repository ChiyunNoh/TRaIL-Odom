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

#ifndef TRAIL_IMU_H
#define TRAIL_IMU_H

// #include "ctraj/core/imu.h"
#include "sensor/imu_frame.h"

namespace trail {
    using IMUFrame = ns_imu::IMUFrame;

    struct IMUFrameArray {
    public:
        using Ptr = std::shared_ptr<IMUFrameArray>;

    private:
        // the timestamp of this array
        double _timestamp;
        std::vector<IMUFrame::Ptr> _frames;

    public:
        explicit IMUFrameArray(double timestamp = INVALID_TIME_STAMP, const std::vector<IMUFrame::Ptr> &frames = {});

        static IMUFrameArray::Ptr
        Create(double timestamp = INVALID_TIME_STAMP, const std::vector<IMUFrame::Ptr> &frames = {});

        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        [[nodiscard]] const std::vector<IMUFrame::Ptr> &GetFrames() const;

        // save radar frames sequence to disk
        static bool SaveFramesArraysToDisk(const std::string &filename,
                                           const std::vector<IMUFrameArray::Ptr> &arrays,
                                           int precision = 10);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("frames", _frames));
        }
    };
}


#endif //TRAIL_IMU_H
