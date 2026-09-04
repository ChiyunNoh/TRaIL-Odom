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

#include <utility>
#include "core/estimator.h"
#include "factor/imu_gyro_factor.hpp"
#include "factor/gravity_factor.hpp"
#include "factor/imu_acce_factor.hpp"
#include "factor/radar_factor.hpp"
#include "factor/bias_factor.hpp"
#include "factor/tail_factor.hpp"
#include "factor/spline_factor.hpp"
#include "factor/rot_prior_factor.hpp"
#include "factor/gravity_stationary_factor.hpp"
#include "factor/lidar_point_to_plane_factor.hpp"

namespace trail {

    std::shared_ptr<ceres::EigenQuaternionManifold> Estimator::QUATER_MANIFOLD(new ceres::EigenQuaternionManifold());
    std::shared_ptr<ceres::SphereManifold<3>> Estimator::GRAVITY_MANIFOLD(new ceres::SphereManifold<3>());

    ceres::Problem::Options Estimator::DefaultProblemOptions() {
        return ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultProblemOptions();
    }

    ceres::Solver::Options Estimator::DefaultSolverOptions(int threadNum, bool toStdout, bool useCUDA) {
        auto defaultSolverOptions = ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultSolverOptions(
                threadNum, toStdout, useCUDA
        );
        if (!useCUDA) {
            defaultSolverOptions.linear_solver_type = ceres::DENSE_SCHUR;
        }
        defaultSolverOptions.trust_region_strategy_type = ceres::DOGLEG;
        // defaultSolverOptions.max_num_iterations = 8;
        defaultSolverOptions.function_tolerance = 1e-12;
        return defaultSolverOptions;
    }

    ceres::Solver::Options Estimator::DefaultSolverOptions_ICP(int threadNum, bool toStdout, bool useCUDA) {
        auto opt = ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>
                    ::DefaultSolverOptions(threadNum, toStdout, useCUDA);

        opt.minimizer_type = ceres::TRUST_REGION;
        opt.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        opt.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        // opt.function_tolerance = 1e-12;

        opt.max_num_iterations = 6;
        opt.minimizer_progress_to_stdout = false;

        return opt;
    }

    ceres::Solver::Options Estimator::DefaultSolverOptions_init(int threadNum, bool toStdout, bool useCUDA) {
        auto opt = ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>
                    ::DefaultSolverOptions(threadNum, toStdout, useCUDA);

        opt.minimizer_type = ceres::TRUST_REGION;
        opt.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        opt.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        opt.function_tolerance = 1e-12;

        opt.max_num_iterations = 1;
        opt.minimizer_progress_to_stdout = false;

        return opt;
    }

    Estimator::Estimator(Configor::Ptr configor, SplineBundleType::Ptr splines,
                         const std::shared_ptr<Eigen::Vector3d> &gravity, const std::shared_ptr<Eigen::Vector3d> &ba,
                         const std::shared_ptr<Eigen::Vector3d> &bg)
            : ceres::Problem(DefaultProblemOptions()), configor(std::move(configor)),
              splines(std::move(splines)), gravity(gravity), ba(ba), bg(bg){}

    Estimator::Ptr Estimator::Create(const Configor::Ptr &configor, const SplineBundleType::Ptr &splines,
                                     const std::shared_ptr<Eigen::Vector3d> &gravity,
                                     const std::shared_ptr<Eigen::Vector3d> &ba,
                                     const std::shared_ptr<Eigen::Vector3d> &bg) {
        return std::make_shared<Estimator>(configor, splines, gravity, ba, bg);
    }

    ceres::Solver::Summary Estimator::Solve(const ceres::Solver::Options &options) {
        ceres::Solver::Summary summary;
        ceres::Solve(options, this, &summary);
        return summary;
    }

    void Estimator::AddRdKnotsData(std::vector<double *> &paramBlockVec,
                                   const Estimator::SplineBundleType::RdSplineType &spline,
                                   const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());

                this->AddParameterBlock(data, 3);
                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }

                // knot recoder
                knotRecoder[reinterpret_cast<long>(&spline)][i]++;
            }
        }
    }

    void Estimator::AddSo3KnotsData(std::vector<double *> &paramBlockVec,
                                    const Estimator::SplineBundleType::So3SplineType &spline,
                                    const Estimator::SplineMetaType &splineMeta, bool setToConst) {
        // for each segment
        for (const auto &seg: splineMeta.segments) {
            // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
            auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

            // from the first control point to the last control point
            for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
                auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());
                // the local parameterization is very important!!!
                this->AddParameterBlock(data, 4, QUATER_MANIFOLD.get());

                paramBlockVec.push_back(data);
                // set this param block to be constant
                if (setToConst) { this->SetParameterBlockConstant(data); }

                // knot recoder
                knotRecoder[reinterpret_cast<long>(&spline)][i]++;
            }
        }
    }


    /**
     * param blocks:
     * [ GRAVITY ]
     */
    void Estimator::AddVelPIMForGravityRecovery(const double dt, const Eigen::Vector3d &deltaVel,
                                                const Eigen::Vector3d &velPim, TRaILOpt option, double weight) {
        auto costFunc = GravityFactor::Create(dt, deltaVel, velPim, weight);

        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

        if (!TRaILOptOption::IsOptionWith(TRaILOpt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity->data());
        }
    }

    ceres::ResidualBlockId Estimator::AddRadarMeasurement(const RadarTarget::Ptr &radarTar, TRaILOpt option, Sophus::SO3d& SO3_RefToW, double weight) {
        auto time = radarTar->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return nullptr;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::PoseSpline)) {
            // if this frame is not in range
            return nullptr;
        }

        // prepare metas for splines
        SplineMetaType so3Meta, poseMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::PoseSpline, {{time, time}}, poseMeta); 

        auto costFunc = RadarFactor<Configor::Prior::SplineOrder>::Create(
                configor->dataStream.CalibParam, SO3_RefToW, radarTar, so3Meta, poseMeta, weight
        );

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // knots of pose spline
        for (int i = 0; i < static_cast<int>(poseMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(1);
        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + poseMeta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_SO3, option)
        );
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::PoseSpline), poseMeta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_POS, option)
        );

        ceres::LossFunction* loss = nullptr;
        loss = new ceres::ScaledLoss(
            new ceres::CauchyLoss(configor->prior.CauchyLossForRadarFactor), 
            weight,                                                         
            ceres::TAKE_OWNERSHIP);

        auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        return id;

    }


    void Estimator::AddRadarProxMeasurement(const RadarTarget::Ptr &radarTar, const Eigen::Vector3d &ref_vel, const Eigen::MatrixXd &degenerate_basis_radar, TRaILOpt option, double weight) {
        auto time = radarTar->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::PoseSpline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta, poseMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta); 
        splines->CalculateRdSplineMeta(Configor::Preference::PoseSpline, {{time, time}}, poseMeta); 

        auto costFunc = RadarFactor_prox<Configor::Prior::SplineOrder>::Create(
                configor->dataStream.CalibParam, ref_vel, degenerate_basis_radar, radarTar, so3Meta, poseMeta, weight
        );

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // knots of pose spline
        for (int i = 0; i < static_cast<int>(poseMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(1);
        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + poseMeta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_SO3, option)
        );
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::PoseSpline), poseMeta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_POS, option)
        );

        this->AddResidualBlock(
                costFunc, nullptr, paramBlockVec
            );



        return;
    }




    /**
     * param blocks:
     * [ SO3 | ... | SO3 | BG ]
     */
    void Estimator::AddGyroMeasurementWithConstBias(const IMUFrame::Ptr &frame, TRaILOpt option, double weight) {
        auto time = frame->GetTimestamp();

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);

        auto costFunc = IMUGyroFactorWithConstBias<Configor::Prior::SplineOrder>::Create(so3Meta, frame, weight);
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // BIAS OF GYRO
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters());


        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_SO3, option)
        );
        paramBlockVec.push_back(bg->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

        if (!TRaILOptOption::IsOptionWith(TRaILOpt::OPT_BG, option)) {
            this->SetParameterBlockConstant(bg->data());
        }
    }

    void Estimator::AddAcceMeasurementWithConstBias(const IMUFrame::Ptr &frame, TRaILOpt option, double weight) {
        auto time = frame->GetTimestamp();
        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::PoseSpline)) {
            // if this frame is not in range
            return;
        }

        SplineMetaType so3Meta, poseMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::PoseSpline, {{time, time}}, poseMeta);

        auto costFunc = IMUAcceFactorWithConstBias<Configor::Prior::SplineOrder>::Create(
                so3Meta, poseMeta, frame, weight
        );


        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // knots of pose spline
        for (int i = 0; i < static_cast<int>(poseMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }
        // BIAS OF ACCE
        costFunc->AddParameterBlock(3);
        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + poseMeta.NumParameters() + 2);
        // knots of so3 spline
        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_SO3, option)
        );
        // knots of pose spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::PoseSpline), poseMeta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_POS, option)
        );
        // BIAS OF ACCE
        paramBlockVec.push_back(ba->data());
        // gravity
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

        if (!TRaILOptOption::IsOptionWith(TRaILOpt::OPT_BA, option)) {
            this->SetParameterBlockConstant(ba->data());
        }
        if (!TRaILOptOption::IsOptionWith(TRaILOpt::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity->data());
        }
        // this->SetParameterBlockConstant(gravity->data());
        // this->SetParameterBlockConstant(ba->data());
    }





    void Estimator::ShowKnotStatus() const {
        for (const auto &[splineAddress, knotInfo]: knotRecoder) {
            std::stringstream stream;
            stream << "spline: " << splineAddress << ", ";
            for (const auto &[knotId, count]: knotInfo) {
                stream << '[' << knotId << ": " << count << "] ";
            }
            spdlog::info("{}", stream.str());
        }
    }

    ceres::ResidualBlockId Estimator::AddAcceBiasPriori(const BiasFilter::StatePack &priori, TRaILOpt option) {
        auto costFunc = BiasFactor::Create(priori.state, priori.var);
        costFunc->AddParameterBlock(3);
        costFunc->SetNumResiduals(3);
        auto id = this->AddResidualBlock(costFunc, nullptr, ba->data());

        if (!TRaILOptOption::IsOptionWith(TRaILOpt::OPT_BA, option)) {
            this->SetParameterBlockConstant(ba->data());
        }
        return id;
    }

    ceres::ResidualBlockId Estimator::AddGyroBiasPriori(const BiasFilter::StatePack &priori, TRaILOpt option) {
        auto costFunc = BiasFactor::Create(priori.state, priori.var);
        costFunc->AddParameterBlock(3);
        costFunc->SetNumResiduals(3);
        auto id = this->AddResidualBlock(costFunc, nullptr, bg->data());

        if (!TRaILOptOption::IsOptionWith(TRaILOpt::OPT_BG, option)) {
            this->SetParameterBlockConstant(bg->data());
        }
        return id;
    }

    std::vector<ceres::ResidualBlockId> Estimator::AddPoseSplineTailConstraint(TRaILOpt option, double weight) {
        auto &poseSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);
        std::vector<ceres::ResidualBlockId> idVec;
        for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j) {
            auto costFunc = RdTailFactor::Create(weight);
            costFunc->AddParameterBlock(3);
            costFunc->AddParameterBlock(3);
            costFunc->AddParameterBlock(3);
            costFunc->SetNumResiduals(3);

            // organize the param block vector
            std::vector<double *> paramBlockVec(3);
            for (int i = 0; i < 3; ++i) {
                paramBlockVec.at(i) = poseSpline.GetKnot(
                        j + i + static_cast<int>(poseSpline.GetKnots().size()) - Configor::Prior::SplineOrder
                ).data();

            }

            auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
            idVec.push_back(id);

            if (!TRaILOptOption::IsOptionWith(TRaILOpt::OPT_POS, option)) {
                for (auto &knot: paramBlockVec) { this->SetParameterBlockConstant(knot); }
            }
        }
        return idVec;
    }

    /**
     * param blocks:
     * [ SO3 | SO3 | SO3 ]
     */
    std::vector<ceres::ResidualBlockId> Estimator::AddSo3SplineTailConstraint(TRaILOpt option, double weight) {
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        std::vector<ceres::ResidualBlockId> idVec;
        for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j) {
            auto costFunc = So3TailFactor::Create(weight);
            costFunc->AddParameterBlock(4);
            costFunc->AddParameterBlock(4);
            costFunc->AddParameterBlock(4);
            costFunc->SetNumResiduals(3);

            // organize the param block vector
            std::vector<double *> paramBlockVec(3);
            for (int i = 0; i < 3; ++i) {
                paramBlockVec.at(i) = so3Spline.GetKnot(
                        j + i + static_cast<int>(so3Spline.GetKnots().size()) - Configor::Prior::SplineOrder
                ).data();
            }

            auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
            idVec.push_back(id);

            for (const auto &item: paramBlockVec) { this->SetManifold(item, QUATER_MANIFOLD.get()); }

            if (!TRaILOptOption::IsOptionWith(TRaILOpt::OPT_SO3, option)) {
                for (auto &knot: paramBlockVec) { this->SetParameterBlockConstant(knot); }
            }
        }
        return idVec;
    }

    /**
     * param blocks:
     * [ POS | ... | POS ]
     */
    void Estimator::AddPoseConstraint(double time1, double time2, const Eigen::Vector3d &value, TRaILOpt option, double weight) {
        if (!splines->TimeInRangeForSo3(time1, Configor::Preference::SO3Spline) ||
            !splines->TimeInRangeForSo3(time2, Configor::Preference::SO3Spline) ||
            !splines->TimeInRangeForRd(time1, Configor::Preference::PoseSpline) ||
            !splines->TimeInRangeForRd(time2, Configor::Preference::PoseSpline)) {
            // if this frame is not in range
            return;
        }
        // prepare metas for splines
        SplineMetaType poseMeta, so3Meta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time2, time2}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::PoseSpline, {{time1, time1},
                                                                         {time2, time2}}, poseMeta);


        auto costFunc = RdSplineFactor<Configor::Prior::SplineOrder>::Create(poseMeta, so3Meta, time1, time2, value, weight);

        // knots of pose spline
        for (int i = 0; i < static_cast<int>(poseMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + poseMeta.NumParameters());

        // knots of pose spline
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::PoseSpline), poseMeta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_POS, option)
        );

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_SO3, option)
        );


        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    }

    void Estimator::AddRotationConstraint(double time1, double time2, const Sophus::SO3d &value, TRaILOpt option, double weight) {
        if (!splines->TimeInRangeForSo3(time1, Configor::Preference::SO3Spline) ||
            !splines->TimeInRangeForSo3(time2, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        // prepare metas for splines
        SplineMetaType so3Meta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time1, time1},
                                                                         {time2, time2}}, so3Meta);

        auto costFunc = So3SplineFactor<Configor::Prior::SplineOrder>::Create(so3Meta, time1, time2, value, weight);
        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters());

        // knots of so3 spline
        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_SO3, option)
        );

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    }

    void Estimator::AddRotPrior(double st, double dt, double weight) {

        if (!splines->TimeInRangeForSo3(st, Configor::Preference::SO3Spline)) {
            return;
        }

        // prepare metas for splines
        SplineMetaType so3Meta, gravMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{st,st},
                                                                          {st+dt, st+dt}}, so3Meta);

        auto costFunc = RotPriorFactor<Configor::Prior::SplineOrder>::Create(st, dt, so3Meta, weight);

        // knots of so3 spline
        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                false
        );


        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        return;

    }

    void Estimator::AddLiDARPointToPlaneFactor(const Estimator::PlaneMatch &match, double weight, TRaILOpt option){
        auto time = match.timestamp;

        if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline)) {
            // if this frame is not in range
            return;
        }
        if (!splines->TimeInRangeForRd(time, Configor::Preference::PoseSpline)) {
            // if this frame is not in range
            return;
        }

        // // prepare metas for splines
        SplineMetaType so3Meta, poseMeta;
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
        splines->CalculateRdSplineMeta(Configor::Preference::PoseSpline, {{time, time}}, poseMeta);

        Eigen::Vector3d p = match.p;
        Eigen::Vector3d q = match.q;
        Eigen::Vector3d n = match.n;

        const double tau_rms = 0.02;       
        double w_rms = std::exp(-0.5 * (match.weight / tau_rms) * (match.weight / tau_rms));

        auto costFunc = LiDARPointToPlaneFactor<Configor::Prior::SplineOrder>::Create(
                p, q, n, time, so3Meta, poseMeta, weight
        );

        for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        for (int i = 0; i < static_cast<int>(poseMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        costFunc->SetNumResiduals(1);
        std::vector<double *> paramBlockVec;
        paramBlockVec.reserve(so3Meta.NumParameters() + poseMeta.NumParameters());

        AddSo3KnotsData(
                paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_SO3, option)
        );
        AddRdKnotsData(
                paramBlockVec, splines->GetRdSpline(Configor::Preference::PoseSpline), poseMeta,
                !TRaILOptOption::IsOptionWith(TRaILOpt::OPT_POS, option)
        );

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

        return;

    }

    void Estimator::AddStationaryGravity(Eigen::Vector3d &acceMean) {
        auto costFunc = GravityStationaryFactor::Create(acceMean, 10);
        // // organize the param block vector
        // gravity
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        paramBlockVec.push_back(gravity->data());

        this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());
        return;
    }

}
