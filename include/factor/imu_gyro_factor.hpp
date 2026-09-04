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

#ifndef TRAIL_IMU_GYRO_FACTOR_HPP
#define TRAIL_IMU_GYRO_FACTOR_HPP

#include <utility>

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace trail {
    template<int Order>
    struct IMUGyroFactorWithConstBias {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        IMUFrame::Ptr frame;

        double weight;
        double so3DtInv;

        std::pair<std::size_t, double> so3IU;
        std::size_t SO3_OFFSET, BG_OFFSET;

    public:
        IMUGyroFactorWithConstBias(const SplineMetaType &so3Meta, IMUFrame::Ptr imuFrame, double weight)
                : frame(std::move(imuFrame)), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt) {
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);

            SO3_OFFSET = so3IU.first;
            BG_OFFSET = so3Meta.NumParameters();
        }

        static auto Create(const SplineMetaType &so3Meta, const IMUFrame::Ptr &frame, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUGyroFactorWithConstBias>(
                    new IMUGyroFactorWithConstBias(so3Meta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUGyroFactorWithConstBias).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | BG ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, nullptr, &ANG_VEL_CurToRefInCur
            );

            Eigen::Map<const Eigen::Vector3<T>> BG(parBlocks[BG_OFFSET]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = ANG_VEL_CurToRefInCur - (frame->GetGyro().cast<T>() - BG);
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif // TRAIL_IMU_GYRO_FACTOR_HPP
