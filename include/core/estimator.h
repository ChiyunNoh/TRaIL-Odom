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

#ifndef TRAIL_ESTIMATOR_H
#define TRAIL_ESTIMATOR_H

#include "ctraj/core/spline_bundle.h"
#include "config/configor.h"
#include "ctraj/core/trajectory_estimator.h"
#include "sensor/imu.h"
#include "sensor/radar.h"
#include "core/bias_filter.h"
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

namespace trail {
    using namespace magic_enum::bitwise_operators;

    struct TRaILOptOption {
        enum class Option : std::uint32_t {
            /**
             * @brief options
             */
            NONE = 1 << 0,
            OPT_SO3 = 1 << 1,
            OPT_POS = 1 << 2,
            OPT_BA = 1 << 3,
            OPT_BG = 1 << 4,
            OPT_GRAVITY = 1 << 5,
            ALL = OPT_SO3 | OPT_POS | OPT_BA | OPT_BG | OPT_GRAVITY
        };

        static bool IsOptionWith(Option desired, Option curOption) {
            return (desired == (desired & curOption));
        }
    };

    using TRaILOpt = TRaILOptOption::Option;

    class Estimator : public ceres::Problem {
    
    public:
        struct PlaneMatch {
            Eigen::Vector3d q;   
            Eigen::Vector3d n; 
            Eigen::Vector3d p;  
            double timestamp;            
            double weight; 
            
            Eigen::VectorXd B;
            std::size_t s;

            bool sampled_valid;
        };

        using PlaneMatches = tbb::concurrent_vector<PlaneMatch>;

        using Ptr = std::shared_ptr<Estimator>;
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        Configor::Ptr configor;
        SplineBundleType::Ptr splines;
        std::shared_ptr<Eigen::Vector3d> gravity;
        std::shared_ptr<Eigen::Vector3d> ba, bg;

        // manifolds
        static std::shared_ptr<ceres::EigenQuaternionManifold> QUATER_MANIFOLD;
        static std::shared_ptr<ceres::SphereManifold<3>> GRAVITY_MANIFOLD;

        // involved knots recoder: [spline, knot, count]
        std::map<long, std::map<std::size_t, int>> knotRecoder;

    public:
        Estimator(Configor::Ptr configor, SplineBundleType::Ptr splines,
                  const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                  const std::shared_ptr<Eigen::Vector3d> &bg);

        static Ptr Create(const Configor::Ptr &configor, const SplineBundleType::Ptr &splines,
                          const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                          const std::shared_ptr<Eigen::Vector3d> &bg);

        static ceres::Problem::Options DefaultProblemOptions();

        static ceres::Solver::Options
        DefaultSolverOptions(int threadNum = -1, bool toStdout = true, bool useCUDA = false);

        static ceres::Solver::Options
        DefaultSolverOptions_ICP(int threadNum = -1, bool toStdout = true, bool useCUDA = false);

        static ceres::Solver::Options
        DefaultSolverOptions_init(int threadNum = -1, bool toStdout = true, bool useCUDA = false);

        ceres::Solver::Summary Solve(const ceres::Solver::Options &options = Estimator::DefaultSolverOptions());

        void ShowKnotStatus() const;

    public:
        void AddGyroMeasurementWithConstBias(const IMUFrame::Ptr &frame, TRaILOpt option, double weight);

        void AddVelPIMForGravityRecovery(double dt, const Eigen::Vector3d &deltaVel, const Eigen::Vector3d &velPim,
                                         TRaILOpt option, double weight);

        void AddAcceMeasurementWithConstBias(const IMUFrame::Ptr &frame, TRaILOpt option, double weight);

        ceres::ResidualBlockId AddRadarMeasurement(const RadarTarget::Ptr &radarTar, TRaILOpt option, Sophus::SO3d& SO3_RefToW, double weight);

        void AddRadarProxMeasurement(const RadarTarget::Ptr &radarTar, const Eigen::Vector3d &ref_vel, const Eigen::MatrixXd &degenerate_basis_radar, TRaILOpt option, double weight);

        void AddLiDARPointToPlaneFactor(const PlaneMatch &match, double weight, TRaILOpt option);

        ceres::ResidualBlockId AddAcceBiasPriori(const BiasFilter::StatePack &priori, TRaILOpt option);

        ceres::ResidualBlockId AddGyroBiasPriori(const BiasFilter::StatePack &priori, TRaILOpt option);

        std::vector<ceres::ResidualBlockId> AddPoseSplineTailConstraint(TRaILOpt option, double weight);

        std::vector<ceres::ResidualBlockId> AddSo3SplineTailConstraint(TRaILOpt option, double weight);

        void AddPoseConstraint(double time1, double time2, const Eigen::Vector3d &value, TRaILOpt option, double weight);

        void AddRotationConstraint(double time1, double time2, const Sophus::SO3d &value, TRaILOpt option, double weight);
        void AddStationaryGravity(Eigen::Vector3d &acceMean);

        void AddRotPrior(double st, double dt, double weight);

    protected:
        void AddSo3KnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::So3SplineType &spline,
                             const SplineMetaType &splineMeta, bool setToConst);

        void AddRdKnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::RdSplineType &spline,
                            const SplineMetaType &splineMeta, bool setToConst);

    };
}

#endif //TRAIL_ESTIMATOR_H
