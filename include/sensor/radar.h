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

#ifndef TRAIL_RADAR_H
#define TRAIL_RADAR_H

#include "memory"
#include "util/utils.hpp"
#include "ctraj/utils/macros.hpp"
#include "ctraj/utils/utils.hpp"

namespace trail {

    struct RadarTarget {
    public:
        using Ptr = std::shared_ptr<RadarTarget>;

    private:
        // the timestamp of this frame
        double _timestamp;

        Eigen::Vector3d _target;
        double _radialVel;

        double _range, _invRange;

    public:
        /**
         * @attention rawMes: [ range | theta | phi | target radial vel with respect to radar in frame {R} ]
         */
        explicit RadarTarget(double timestamp = INVALID_TIME_STAMP,
                             const Eigen::Vector4d &rawMes = Eigen::Vector4d::Zero());

        /**
         * @attention rawMes: [ xyz | target radial vel with respect to radar in frame {R} ]
         */
        explicit RadarTarget(double timestamp, Eigen::Vector3d target, double radialVel);

        /**
         * @attention rawMes: [ range | theta | phi | target radial vel with respect to radar in frame {R} ]
         */
        static RadarTarget::Ptr Create(double timestamp = INVALID_TIME_STAMP,
                                       const Eigen::Vector4d &rawMes = Eigen::Vector4d::Zero());

        /**
         * @attention rawMes: [ xyz | target radial vel with respect to radar in frame {R} ]
         */
        static RadarTarget::Ptr Create(double timestamp, const Eigen::Vector3d &target, double radialVel);

        // access
        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        [[nodiscard]] const Eigen::Vector3d &GetTargetXYZ() const;

        [[nodiscard]]  Eigen::Vector3d GetTargetRTP() const;

        [[nodiscard]]  double GetRadialVelocity() const;

        [[nodiscard]] double GetRange() const;

        [[nodiscard]] double GetInvRange() const;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    cereal::make_nvp("timestamp", _timestamp),
                    cereal::make_nvp("target", _target),
                    cereal::make_nvp("radial_vel", _radialVel)
            );
        }
    };

    struct RadarTargetArray {
    public:
        using Ptr = std::shared_ptr<RadarTargetArray>;

    private:
        // the timestamp of this array
        double _timestamp;
        std::vector<RadarTarget::Ptr> _targets;

    public:
        explicit RadarTargetArray(double timestamp = INVALID_TIME_STAMP,
                                  const std::vector<RadarTarget::Ptr> &targets = {});

        static RadarTargetArray::Ptr
        Create(double timestamp = INVALID_TIME_STAMP, const std::vector<RadarTarget::Ptr> &targets = {});

        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        [[nodiscard]] const std::vector<RadarTarget::Ptr> &GetTargets() const;

        // save radar frames sequence to disk
        static bool SaveTargetArraysToDisk(const std::string &filename,
                                           const std::vector<RadarTargetArray::Ptr> &arrays,
                                           int precision = 10);

        Eigen::Vector3d RadarVelocityFromStaticTargetArray(const Sophus::SO3d &SO3_RtoB0) const;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("targets", _targets));
        }
    };
}


#endif //TRAIL_RADAR_H
