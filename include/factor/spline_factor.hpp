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

#ifndef TRAIL_SPLINE_FACTOR_HPP
#define TRAIL_SPLINE_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace trail {
    template<int Order>
    struct RdSplineFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        double timestamp;
        Eigen::Vector3d value;

        double weight;
        double poseDtInv;
        double so3DtInv;

        std::pair<std::size_t, double> poseIU1, poseIU2, so3IU;
        std::size_t POSE_OFFSET1, POSE_OFFSET2, SO3_OFFSET;

    public:
        RdSplineFactor(const SplineMetaType &poseMeta, const SplineMetaType &so3Meta, double time1, double time2, Eigen::Vector3d value, double weight)
                : timestamp(timestamp), value(std::move(value)), weight(weight),
                  poseDtInv(1.0 / poseMeta.segments.front().dt), so3DtInv(1.0 / so3Meta.segments.front().dt) {
            // compute knots indexes
            poseMeta.template ComputeSplineIndex(time1, poseIU1.first, poseIU1.second);
            poseMeta.template ComputeSplineIndex(time2, poseIU2.first, poseIU2.second);
            so3Meta.template ComputeSplineIndex(time2, so3IU.first, so3IU.second);

            // compute knots offset in 'parBlocks'
            POSE_OFFSET1 = poseIU1.first;
            POSE_OFFSET2 = poseIU2.first;
            SO3_OFFSET = poseMeta.NumParameters() + so3IU.first;
        }

        static auto
        Create(const SplineMetaType &poseMeta, const SplineMetaType &so3Meta, double time1, double time2, const Eigen::Vector3d &value, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RdSplineFactor>(
                    new RdSplineFactor(poseMeta, so3Meta, time1, time2, value, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RdSplineFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ POS | ... | POS ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Vector3<T> POSE_CurToRef1, POSE_CurToRef2;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + POSE_OFFSET1, poseIU1.second, poseDtInv, &POSE_CurToRef1
            );
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + POSE_OFFSET2, poseIU2.second, poseDtInv, &POSE_CurToRef2
            );

            Sophus::SO3<T> SO3_CurToRef2;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef2
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = SO3_CurToRef2.inverse() * (POSE_CurToRef1 - POSE_CurToRef2) - value.cast<T>(); // delta p = R2^T (p1 - p2)
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<int Order>
    struct So3SplineFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        double timestamp;
        Sophus::SO3d value;

        double weight;
        double so3DtInv;

        std::pair<std::size_t, double> so3IU1, so3IU2;
        std::size_t SO3_OFFSET1, SO3_OFFSET2;

    public:
        So3SplineFactor(const SplineMetaType &so3Meta, double time1, double time2, const Sophus::SO3d &value, double weight)
                : timestamp(timestamp), value(value), weight(weight), so3DtInv(1.0 / so3Meta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(time1, so3IU1.first, so3IU1.second);
            so3Meta.template ComputeSplineIndex(time2, so3IU2.first, so3IU2.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET1 = so3IU1.first;
            SO3_OFFSET2 = so3IU2.first;
        }

        static auto Create(const SplineMetaType &so3Meta, double time1, double time2, const Sophus::SO3d &value, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<So3SplineFactor>(
                    new So3SplineFactor(so3Meta, time1, time2, value, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(So3SplineFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute rotation from {current frame} to {reference frame}
            Sophus::SO3<T> SO3_CurToRef1, SO3_CurToRef2;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET1, so3IU1.second, so3DtInv, &SO3_CurToRef1
            );
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET2, so3IU2.second, so3DtInv, &SO3_CurToRef2
            );

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = (SO3_CurToRef1.inverse() * SO3_CurToRef2 * value.cast<T>()).log();
            residuals = T(weight) * residuals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //TRAIL_SPLINE_FACTOR_HPP
