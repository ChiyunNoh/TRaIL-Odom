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

#ifndef TRAIL_BIAS_FILTER_H
#define TRAIL_BIAS_FILTER_H

#include <ostream>
#include "ctraj/core/spline_bundle.h"
#include <cereal/types/list.hpp>

namespace trail {
    class BiasFilter {
    public:
        using Ptr = std::shared_ptr<BiasFilter>;

        struct StatePack {
            double time{};
            Eigen::Vector3d state;
            Eigen::Matrix3d var;

            StatePack(double time, Eigen::Vector3d state, const Eigen::Vector3d &varMat);

            StatePack();

            friend std::ostream &operator<<(std::ostream &os, const StatePack &pack);

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(time), CEREAL_NVP(state), CEREAL_NVP(var));
            }
        };

    private:
        StatePack curState;
        const double sigma2;

        std::list<StatePack> stateRecords;

    public:
        BiasFilter(StatePack init, double randomWalk);

        static Ptr Create(const StatePack &init, double randomWalk);

        [[nodiscard]] StatePack Prediction(double t) const;

        [[nodiscard]] const StatePack &GetCurState() const;

        [[nodiscard]] const std::list<StatePack> &GetStateRecords() const;

        void Update(const StatePack &mes);

        void UpdateByEstimator(const StatePack &est);
    };
}


#endif //TRAIL_BIAS_FILTER_H
