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

#ifndef TRAIL_RADAR_FACTOR_HPP
#define TRAIL_RADAR_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "core/calib_param_manager.h"
#include "sensor/imu.h"

namespace trail {
    template<int Order>
    struct RadarFactor {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        const CalibParamManager &parMagr;
        RadarTarget::Ptr target;
        Sophus::SO3d SO3_RefToW;

        double weight;
        double so3DtInv, poseDtInv;

        // compute knots indexes
        std::pair<std::size_t, double> so3IU, poseIU;
        std::size_t SO3_OFFSET, POSE_OFFSET;

    public:
        RadarFactor(const CalibParamManager &calibParMagr, Sophus::SO3d& SO3_RefToW, RadarTarget::Ptr radarTar, const SplineMetaType &so3Meta,
                    const SplineMetaType &poseMeta, double weight)
                : parMagr(calibParMagr), target(std::move(radarTar)), SO3_RefToW(SO3_RefToW), weight(weight),
                  so3DtInv(1.0 / so3Meta.segments.front().dt), poseDtInv(1.0 / poseMeta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(target->GetTimestamp(), so3IU.first, so3IU.second);
            poseMeta.template ComputeSplineIndex(target->GetTimestamp(), poseIU.first, poseIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            POSE_OFFSET = so3Meta.NumParameters() + poseIU.first;
        }

        static auto Create(const CalibParamManager &calibParMagr, Sophus::SO3d& SO3_RefToW, const RadarTarget::Ptr &radarTar,
                           const SplineMetaType &so3Meta, const SplineMetaType &poseMeta, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RadarFactor>(
                    new RadarFactor(calibParMagr, SO3_RefToW, radarTar, so3Meta, poseMeta, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RadarFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | POS | ... | POS ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute rotation from {current frame} to {reference frame}
            Sophus::SO3<T> SO3_CurToRef;
            // compute angular velocity of {current frame} with respect to {reference frame} expressed in {current frame}
            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef, &ANG_VEL_CurToRefInCur
            );
            // Eigen::Vector3<T> ANG_VEL_CurToRefInRef = SO3_CurToRef * ANG_VEL_CurToRefInCur;

            // compute linear velocity of {current frame} with respect to {reference frame} expressed in {reference frame}
            // the first derivative of the position spline is the linear velocity
            Eigen::Vector3<T> LIN_VEL_CurToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 1>(
                    parBlocks + POSE_OFFSET, poseIU.second, poseDtInv, &LIN_VEL_CurToRefInRef
            );

            Eigen::Vector3<T> LIN_VEL_RtoRefInCur =
                    Sophus::SO3<T>::hat(ANG_VEL_CurToRefInCur ) * parMagr.POS_RinB +
                    SO3_CurToRef.matrix().transpose() *LIN_VEL_CurToRefInRef; //V^w_wr

            T v1 = -target->GetTargetXYZ().cast<T>().dot(
                    parMagr.SO3_RtoB.matrix().transpose() * LIN_VEL_RtoRefInCur
            );

            T v2 = static_cast<T>(target->GetRadialVelocity());

            Eigen::Map<Eigen::Vector1<T>> residuals(sResiduals);
            // cout<<"Radar : "<<target->GetInvRange() * v1 - v2<<endl;
            residuals(0, 0) = T(weight) * (target->GetInvRange() * v1 - v2);

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<int Order>
    struct RadarFactor_prox {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        const CalibParamManager &parMagr;
        RadarTarget::Ptr target;
        const Eigen::Vector3d &ref_vel;
        const Eigen::MatrixXd &degenerate_basis_radar;

        double weight;
        double so3DtInv, poseDtInv;

        // compute knots indexes
        std::pair<std::size_t, double> so3IU, poseIU;
        std::size_t SO3_OFFSET, POSE_OFFSET;

    public:
        RadarFactor_prox(const CalibParamManager &calibParMagr, const Eigen::Vector3d &ref_vel, const Eigen::MatrixXd &degenerate_basis_radar, RadarTarget::Ptr radarTar, const SplineMetaType &so3Meta,
                    const SplineMetaType &poseMeta, double weight)
                : parMagr(calibParMagr), target(std::move(radarTar)), ref_vel(ref_vel), degenerate_basis_radar(degenerate_basis_radar), weight(weight),
                  so3DtInv(1.0 / so3Meta.segments.front().dt), poseDtInv(1.0 / poseMeta.segments.front().dt) {
            // compute knots indexes
            so3Meta.template ComputeSplineIndex(target->GetTimestamp(), so3IU.first, so3IU.second);
            poseMeta.template ComputeSplineIndex(target->GetTimestamp(), poseIU.first, poseIU.second);

            // compute knots offset in 'parBlocks'
            SO3_OFFSET = so3IU.first;
            POSE_OFFSET = so3Meta.NumParameters() + poseIU.first;
        }

        static auto Create(const CalibParamManager &calibParMagr, const Eigen::Vector3d &ref_vel, const Eigen::MatrixXd &degenerate_basis_radar, const RadarTarget::Ptr &radarTar,
                           const SplineMetaType &so3Meta, const SplineMetaType &poseMeta, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RadarFactor_prox>(
                    new RadarFactor_prox(calibParMagr, ref_vel, degenerate_basis_radar, radarTar, so3Meta, poseMeta, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RadarFactor_prox).hash_code();
        }

    public:

        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            // compute rotation from {current frame} to {reference frame}
            Sophus::SO3<T> SO3_CurToRef;
            // compute angular velocity of {current frame} with respect to {reference frame} expressed in {current frame}
            Sophus::SO3Tangent<T> ANG_VEL_CurToRefInCur;
            ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
                    parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_CurToRef, &ANG_VEL_CurToRefInCur
            );
            // Eigen::Vector3<T> ANG_VEL_CurToRefInRef = SO3_CurToRef * ANG_VEL_CurToRefInCur;

            // compute linear velocity of {current frame} with respect to {reference frame} expressed in {reference frame}
            // the first derivative of the position spline is the linear velocity
            Eigen::Vector3<T> LIN_VEL_CurToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 1>(
                    parBlocks + POSE_OFFSET, poseIU.second, poseDtInv, &LIN_VEL_CurToRefInRef
            );

            Eigen::Vector3<T> LIN_VEL_RtoRefInCur =
                    Sophus::SO3<T>::hat(ANG_VEL_CurToRefInCur ) * parMagr.POS_RinB +
                    SO3_CurToRef.matrix().transpose() *LIN_VEL_CurToRefInRef; //V^w_wr

            const auto B = degenerate_basis_radar.template cast<T>(); //m *3 

            const auto R_RtoB_T = parMagr.SO3_RtoB.matrix().template cast<T>().transpose();

            T v1 = -target->GetTargetXYZ().cast<T>().dot(
                    B.transpose() * B * R_RtoB_T * (LIN_VEL_RtoRefInCur - ref_vel.cast<T>())
            );
            
            Eigen::Map<Eigen::Vector1<T>> residuals(sResiduals);
            // cout<<"Radar : "<<target->GetInvRange() * v1 - v2<<endl;
            residuals(0, 0) = T(weight) * (target->GetInvRange() * v1) ;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}
#endif //TRAIL_RADAR_FACTOR_HPP
