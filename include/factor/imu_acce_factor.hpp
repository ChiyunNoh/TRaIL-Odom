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

#ifndef TRAIL_IMU_ACCE_FACTOR_HPP
#define TRAIL_IMU_ACCE_FACTOR_HPP

#include <utility>

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace trail {
    template<int Order>
    struct IMUAcceFactorWithConstBias {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        IMUFrame::Ptr frame;

        double weight;
        double so3DtInv, poseDtInv;

        std::pair<std::size_t, double> so3IU, poseIU;

        std::size_t SO3_OFFSET, POSE_OFFSET, BA_OFFSET, GRAVITY_OFFSET;

    public:
        IMUAcceFactorWithConstBias(const SplineMetaType &so3Meta, const SplineMetaType &poseMeta,
                                   IMUFrame::Ptr imuFrame, double weight)
                : frame(std::move(imuFrame)), weight(weight),
                  so3DtInv(1.0 / so3Meta.segments.front().dt),
                  poseDtInv(1.0 / poseMeta.segments.front().dt) {
            so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);
            poseMeta.template ComputeSplineIndex(frame->GetTimestamp(), poseIU.first, poseIU.second);

            SO3_OFFSET = so3IU.first;
            POSE_OFFSET = so3Meta.NumParameters() + poseIU.first;
            BA_OFFSET = so3Meta.NumParameters() + poseMeta.NumParameters();
            GRAVITY_OFFSET = BA_OFFSET + 1;
        }

        static auto Create(const SplineMetaType &so3Meta, const SplineMetaType &poseMeta,
                           const IMUFrame::Ptr &frame, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUAcceFactorWithConstBias>(
                    new IMUAcceFactorWithConstBias(so3Meta, poseMeta, frame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUAcceFactorWithConstBias).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS | BA | GRAVITY ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Sophus::SO3<T> SO3_CurToRef;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef
            );

            Eigen::Vector3<T> acc_CurToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 2>(
                    parBlocks + POSE_OFFSET, poseIU.second, poseDtInv, &acc_CurToRefInRef
            );

            Eigen::Map<const Eigen::Vector3<T>> BA(parBlocks[BA_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[GRAVITY_OFFSET]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = SO3_CurToRef.inverse() * (acc_CurToRefInRef - GRAVITY_IN_REF) + BA
                        - frame->GetAcce().cast<T>();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif // TRAIL_IMU_ACCE_FACTOR_HPP
