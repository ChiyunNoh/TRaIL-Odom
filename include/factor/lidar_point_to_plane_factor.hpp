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

#ifndef TRAIL_LIDAR_P2P_HPP
#define TRAIL_LIDAR_P2P_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"

namespace trail {
    template<int Order>
    struct LiDARPointToPlaneFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;
    private:
        double so3DtInv, poseDtInv;

        // array offset
        Eigen::Vector3d p, q, n;
        std::pair<std::size_t, double> so3IU, poseIU;
        std::size_t SO3_OFFSET, POSE_OFFSET;

        double weight;

    public:
        LiDARPointToPlaneFactor(Eigen::Vector3d& p, Eigen::Vector3d& q, Eigen::Vector3d& n, double time, const SplineMetaType &so3Meta, const SplineMetaType &poseMeta, double weight)
                : so3DtInv(1.0 / so3Meta.segments.front().dt), poseDtInv(1.0 / poseMeta.segments.front().dt), 
                p(p), q(q), n(n), weight(weight){
                // compute knots indexes
                so3Meta.template ComputeSplineIndex(time, so3IU.first, so3IU.second);
                poseMeta.template ComputeSplineIndex(time, poseIU.first, poseIU.second);    

                SO3_OFFSET = so3IU.first;
                POSE_OFFSET = so3Meta.NumParameters() + poseIU.first;
            }


        static auto
        Create(Eigen::Vector3d& p, Eigen::Vector3d& q, Eigen::Vector3d& n, double time, const SplineMetaType &so3Meta, const SplineMetaType &poseMeta, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<LiDARPointToPlaneFactor>(
                    new LiDARPointToPlaneFactor(p, q, n, time, so3Meta, poseMeta, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(LiDARPointToPlaneFactor).hash_code();
        }

    public:
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {

            Sophus::SO3<T> SO3_CurToRef;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef
            );

            Eigen::Vector3<T> POSE_CurToRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    parBlocks + POSE_OFFSET, poseIU.second, poseDtInv, &POSE_CurToRef
            );

            Eigen::Map<Eigen::Vector1<T>> residuals(sResiduals);
            residuals(0, 0) = T(weight) * (SO3_CurToRef * p.cast<T>() + POSE_CurToRef - q.cast<T>()).dot(n.cast<T>());

            return true;
        }
    };
}
#endif 
