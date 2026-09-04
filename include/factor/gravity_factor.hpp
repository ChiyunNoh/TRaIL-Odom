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

#ifndef TRAIL_GRAVITY_FACTOR_HPP
#define TRAIL_GRAVITY_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace trail {
    struct GravityFactor {
    private:
        const double dt;
        Eigen::Vector3d deltaVel;
        Eigen::Vector3d velPIM;
        double weight;

    public:
        GravityFactor(const double dt, Eigen::Vector3d deltaVel, Eigen::Vector3d velPim, double weight)
                : dt(dt), deltaVel(std::move(deltaVel)), velPIM(std::move(velPim)), weight(weight) {}

        static auto
        Create(const double dt, const Eigen::Vector3d &deltaVel, const Eigen::Vector3d &velPim, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<GravityFactor>(
                    new GravityFactor(dt, deltaVel, velPim, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(GravityFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ GRAVITY ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> gravity(sKnots[0]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = deltaVel - gravity * dt - velPIM;
            residuals = T(weight) * residuals;

            return true;
        }
    };
}
#endif //TRAIL_GRAVITY_FACTOR_HPP
