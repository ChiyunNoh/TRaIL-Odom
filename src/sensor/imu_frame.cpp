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

#include "sensor/imu_frame.h"
#include "utility"

namespace ns_imu {

IMUFrame::IMUFrame(double timestamp, Eigen::Vector3d gyro, Eigen::Vector3d acce, Eigen::Quaternion<double> orientation)
    : _timestamp(timestamp),
      _gyro(std::move(gyro)),
      _acce(std::move(acce)),
      _orientaion(std::move(orientation)) {}

IMUFrame::Ptr IMUFrame::Create(double timestamp,
                               const Eigen::Vector3d &gyro,
                               const Eigen::Vector3d &acce,
                               const Eigen::Quaternion<double> &orientation) {
    return std::make_shared<IMUFrame>(timestamp, gyro, acce, orientation);
}

double IMUFrame::GetTimestamp() const { return _timestamp; }

const Eigen::Vector3d &IMUFrame::GetGyro() const { return _gyro; }

const Eigen::Vector3d &IMUFrame::GetAcce() const { return _acce; }

const Eigen::Quaternion<double> &IMUFrame::GetOrientation() const {return _orientaion; }

void IMUFrame::SetOrientation(Eigen::Quaternion<double> &orient){
    this->_orientaion = orient;
}

void IMUFrame::SetGyro(Eigen::Vector3d &orient){
    this->_gyro = orient;
}

void IMUFrame::SetAcce(Eigen::Vector3d &acc){
    this->_acce = acc;
}

std::ostream &operator<<(std::ostream &os, const IMUFrame &frame) {
    os << "timestamp: " << frame._timestamp << ", gyro: " << frame._gyro.transpose()
       << ", acce: " << frame._acce.transpose() << ", orientation: "<<frame._orientaion.coeffs().transpose();
    return os;
}

void IMUFrame::SetTimestamp(double timestamp) { _timestamp = timestamp; }

bool IMUFrame::SaveFramesToDisk(const std::string &filename,
                                const std::vector<IMUFrame::Ptr> &frames,
                                int precision) {
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(precision);
    cereal::JSONOutputArchive ar(file);
    ar(cereal::make_nvp("imu_frames", frames));
    return true;
}

}  // namespace ns_ctraj