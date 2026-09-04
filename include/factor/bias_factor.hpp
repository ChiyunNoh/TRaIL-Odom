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

#ifndef TRAIL_BIAS_FACTOR_HPP
#define TRAIL_BIAS_FACTOR_HPP

namespace trail {

    struct BiasFactor {
    private:
        Eigen::Vector3d biasPriori;
        Eigen::Matrix3d weight;

    public:
        BiasFactor(Eigen::Vector3d biasPriori, const Eigen::Matrix3d &var) : biasPriori(std::move(biasPriori)) {
            weight = Eigen::LLT<Eigen::Matrix3d>(var.inverse()).matrixL().transpose();
        }

        static auto Create(const Eigen::Vector3d &biasPriori, const Eigen::Matrix3d &var) {
            return new ceres::DynamicAutoDiffCostFunction<BiasFactor>(new BiasFactor(biasPriori, var));
        }

        static std::size_t TypeHashCode() {
            return typeid(BiasFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ bias ]
         */
        template<class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const {
            Eigen::Map<const Eigen::Vector3<T>> bias(parBlocks[0]);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = bias - biasPriori;
            residuals = weight * residuals;
            // cout<<"Acce Residual : "<<residuals<<endl;

            return true;
        }
    };
}

#endif //TRAIL_BIAS_FACTOR_HPP
