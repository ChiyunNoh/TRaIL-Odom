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

#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include "filesystem"
#include "fstream"
#include "memory"
#include "ostream"
#include "ctraj/utils/macros.hpp"
#include "ctraj/utils/eigen_utils.hpp"
#include "utility"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_imu {

struct IMUFrame {
public:
    using Ptr = std::shared_ptr<IMUFrame>;

private:
    // the timestamp of this imu frame
    double _timestamp;
    // Gyro output
    Eigen::Vector3d _gyro;
    // Accelerometer output
    Eigen::Vector3d _acce;
    // Orientation output (w, x, y, z)
    Eigen::Quaternion<double> _orientaion;  

public:
    // constructor
    explicit IMUFrame(double timestamp = INVALID_TIME_STAMP,
                      Eigen::Vector3d gyro = Eigen::Vector3d::Zero(),
                      Eigen::Vector3d acce = Eigen::Vector3d::Zero(),
                      Eigen::Quaternion<double> orientation = Eigen::Quaternion<double>::Identity());

    // creator
    static IMUFrame::Ptr Create(double timestamp = INVALID_TIME_STAMP,
                                const Eigen::Vector3d &gyro = Eigen::Vector3d::Zero(),
                                const Eigen::Vector3d &acce = Eigen::Vector3d::Zero(),
                                const Eigen::Quaternion<double> &orientation = Eigen::Quaternion<double>::Identity());

    // access
    [[nodiscard]] double GetTimestamp() const;

    [[nodiscard]] const Eigen::Vector3d &GetGyro() const;

    [[nodiscard]] const Eigen::Vector3d &GetAcce() const;

    [[nodiscard]] const Eigen::Quaternion<double> &GetOrientation() const;

    void SetOrientation(Eigen::Quaternion<double> &orient);

    void SetGyro(Eigen::Vector3d &orient);

    void SetAcce(Eigen::Vector3d &acc);

    void SetTimestamp(double timestamp);

    friend std::ostream &operator<<(std::ostream &os, const IMUFrame &frame);

    // save imu frames sequence to disk
    static bool SaveFramesToDisk(const std::string &filename,
                                 const std::vector<IMUFrame::Ptr> &frames,
                                 int precision = 10);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    template <class Archive>
    void serialize(Archive &ar) {
        ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("linear_acce", _acce),
           cereal::make_nvp("angular_velo", _gyro), cereal::make_nvp("Orientation", _orientaion));
    }
};

}  // namespace ns_imu

#endif  // IMU_FRAME_H