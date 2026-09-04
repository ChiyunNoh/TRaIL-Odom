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

#include "core/state_manager.h"

#include <utility>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include "core/estimator.h"
#include "spdlog/stopwatch.h"
#include <spdlog/fmt/ostr.h>

namespace trail {

    // -------------------
    // static member field
    // -------------------
    std::mutex StateManager::StatesMutex = {};
    double st = 0;
    long update_start_index = -1;
    long update_end_index = -1;

    StateManager::StateManager(DataManager::Ptr dataMagr, Configor::Ptr configor, std::shared_ptr<ns_river::VoxelHashMap> voxel_map)
            : dataMagr(std::move(dataMagr)), configor(std::move(configor)), local_map_(std::move(voxel_map)), splines(nullptr),
              gravity(std::make_shared<Eigen::Vector3d>(0.0, 0.0, -this->configor->prior.GravityNorm)),
              ba(std::make_shared<Eigen::Vector3d>(0.0, 0.0, 0.0)),
              bg(std::make_shared<Eigen::Vector3d>(0.0, 0.0, 0.0)),
              margInfo(nullptr) {

              }

    StateManager::Ptr StateManager::Create(const DataManager::Ptr &dataMagr, const Configor::Ptr &configor, const std::shared_ptr<ns_river::VoxelHashMap> &voxel_map) {
        return std::make_shared<StateManager>(dataMagr, configor, voxel_map);
    }

    void StateManager::Run() {
        spdlog::info("'StateManager::Run' has been booted, thread id: {}.", TRAIL_TO_STR(std::this_thread::get_id()));

        rclcpp::Rate rate(configor->preference.IncrementalOptRate);

        while (rclcpp::ok()) {
            auto status = TRaILStatus::GetStatusPackSafely();
            if (TRaILStatus::IsWith(TRaILStatus::StateManager::Status::ShouldQuit, status.StateMagr)) {
                spdlog::warn("'StateManager::Run' quits normally.");
                break;
            }

            if (!TRaILStatus::IsWith(TRaILStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                if (TRaILStatus::IsWith(TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr)) {
                    {
                        LOCK_TRAIL_STATUS
                        TRaILStatus::DataManager::CurStatus ^= TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady;
                    }
                    spdlog::stopwatch sw;
                    auto valid = TryPerformInitialization();
                    if (valid) {
                        spdlog::info("total time elapsed in initialization: {} (s).", sw);
                    }
                } else {
                    TryPerformICP();
                    spdlog::info("waiting for radar target arrays to perform initialization...");
                }
            } else {
                spdlog::stopwatch sw;
                // perform incremental optimization
                auto valid = IncrementalOptimization(status);
                if (valid) {
                    spdlog::info("--------------- end of incremental optimization ----------------");
                    spdlog::info("total time elapsed in current incremental optimization: {} (s).", sw);
                }
            }

            rate.sleep();
            // ros::spinOnce();
        }
    }

    void StateManager::buildRadarSensitivityMatrix(const Eigen::MatrixXd &degenerate_basis_radar, // degenerate_dim x 3
                                                 std::list<RadarTarget::Ptr> &inlier_radarData, // N
                                                 Eigen::MatrixXd &Z // degenerate_dim x N
                                                 ) {
        int i = 0;
        for (const auto &tar: inlier_radarData) {
            Eigen::Vector3d u = tar->GetTargetXYZ() / tar->GetTargetXYZ().norm();
            Z.col(i) = degenerate_basis_radar * u; // degenerate_dim x 1
            i++;
        }
    }

    bool StateManager::isRadarSubspaceObservable(const Eigen::MatrixXd &Z) {
        int degenerate_dim = static_cast<int>(Z.rows());
        if(degenerate_dim == 1) {
            return true; // No degeneracy implies observability
        }
        else {
            const Eigen::MatrixXd M = Z * Z.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(M);
            if (es.info() != Eigen::Success) {
                return false;
            }

            const Eigen::VectorXd evals = es.eigenvalues();
            spdlog::info("Radar subspace eigenvalues: {}", evals.transpose());
            const double max_eval = evals.cwiseAbs().maxCoeff();

            const double abs_tol = 1e-12;
            const double rel_tol = 1e-8;
            const double tol = std::max(abs_tol, rel_tol * max_eval);

            const int rank = static_cast<int>((evals.array() > tol).count());

            return (rank == degenerate_dim);
        }

        return true;
    }

    void StateManager::reweightingRadarMeasurement(std::vector<double> &radar_weight_normalize,
                                                const Eigen::MatrixXd &Z, // degenerate_dim x N
                                                double eta) {
        int degenerate_dim = static_cast<int>(Z.rows());
        int num_measurements = static_cast<int>(Z.cols());

        for(int i = 0; i < num_measurements; ++i) {
            double score = Z.col(i).squaredNorm(); // dot product between radar point and degenerate subspace
            radar_weight_normalize[i] = radar_weight_normalize[i] * std::exp(eta * score);
        }

        //normalize weights to have average 1
        double sum_w = 0.0;
        for (double w : radar_weight_normalize) sum_w += w;

        const int N = static_cast<int>(radar_weight_normalize.size());
        if (sum_w > 1e-12 && N > 0) {
            const double scale = static_cast<double>(N) / sum_w;
            for (double &w : radar_weight_normalize) w *= scale;
        }
    }

    StateManager::PointsTsTuple StateManager::Voxelize(const std::vector<Eigen::Vector3d> &frame, const std::vector<double> &timestamps) const {
        const auto voxel_size = configor->voxelMap.VoxelSize;
        const auto [frame_downsample, frame_downsample_ts] = ns_river::VoxelDownsample(frame, timestamps, voxel_size * 0.4);
        const auto [source, source_ts] = ns_river::VoxelDownsample(frame_downsample, frame_downsample_ts, voxel_size * 1.5);

        return std::make_tuple(
            StateManager::PointsTsPair{std::move(frame_downsample), std::move(frame_downsample_ts)},
            StateManager::PointsTsPair{std::move(source),           std::move(source_ts)}
        );
        //frame_downsample : for building local map
        //source : for data association
    }

    void StateManager::TryPerformICP() {

        auto lidardataseq = dataMagr->GetLidarDataSeq();

        auto radardataseq = dataMagr->GetRadarDataSeq();

        if(lidardataseq.size() > last_lidar) // new lidar frame has come
        {
            last_lidar = lidardataseq.size();
            if(radardataseq.size() > 0 && lidardataseq.size() > 1)
            {
                size_t idx = lidar_scan_times.size() - 1;

                spdlog::stopwatch sw_icp;

                auto nxt = std::prev(lidardataseq.end()); 
                auto it  = std::prev(nxt);                

                Rv.push_back(Sophus::SO3d{});
                Tv.push_back(Eigen::Vector3d::Zero());
                lidar_scan_times.push_back(0.0);

                {
                    auto points_1 = PointCloud2ToEigen((*it)->GetScan()); //std::vector<Eigen::Vector3d>
                    auto timestamps_1 = GetTimestamps((*it)->GetScan());
                    lidar_scan_times[idx] = (*it)->GetTimestamp().first;
                    lidar_scan_times[idx+1] = (*nxt)->GetTimestamp().first;

                    auto points_2 = PointCloud2ToEigen((*nxt)->GetScan());
                    auto timestamps_2 = GetTimestamps((*nxt)->GetScan());

                    // 1. motion compensate the prev lidar frame 
                    Sophus::SE3d prev_pose(Rv[idx], Tv[idx]);

                    // motion compensate using KISS ICP idea
                    const std::vector<Eigen::Vector3d> &deskewed_frame = [&]() {
                        const auto &[min, max] = std::minmax_element(timestamps_1.cbegin(), timestamps_1.cend());
                        const double min_time = *min;
                        const double max_time = *max;
                        const auto normalize = [&](const double t) {
                            return (t - min_time) / (max_time - min_time);
                        };
                        const auto &omega = prev_pose.log();
                        std::vector<Eigen::Vector3d> deskewed_frame(points_1.size());
                        tbb::parallel_for(
                            // Index Range
                            tbb::blocked_range<size_t>{0, deskewed_frame.size()},
                            // Parallel Compute
                            [&](const tbb::blocked_range<size_t> &r) {
                                for (size_t idx = r.begin(); idx < r.end(); ++idx) {
                                    const auto &point = points_1.at(idx);
                                    const auto &stamp = normalize(timestamps_1.at(idx));
                                    const auto pose = Sophus::SE3d::exp((stamp - 1.0) * omega);
                                    deskewed_frame.at(idx) = pose * point;
                                };
                            });
                        return deskewed_frame;
                    }();


                    // 2. downsample and voxelize the prev lidar frame
                    const auto& [frame_pair_1, source_pair_1] = Voxelize(points_1, timestamps_1);
                    const auto& [frame_downsample_1, frame_downsample_ts_1] = frame_pair_1;
                    const auto& [source_1, source_ts_1] = source_pair_1;


                    // 3. ICP between the prev lidar frame and the current lidar frame (using points_2 as local map, source_1 as source)
                    // Compute initial_guess for ICP
                    const Sophus::SE3d initial_guess(Rv[idx], Tv[idx]);
                    const double sigma = 0.5 / 3.0;

                    // // Run point to plane ICP
                    std::vector<Eigen::Vector3d> map_points_bodyframe;
                    map_points_bodyframe.reserve(points_2.size());
                    auto target_pair = StateManager::PointsTsPair{std::move(points_2), std::move(timestamps_2)};
                    TransformLidarPointsToIMUFrame(target_pair, map_points_bodyframe);

                    local_map_->Update(map_points_bodyframe, Sophus::SE3d{});
                    auto new_pose = AlignPointToPlane(source_pair_1, initial_guess, sigma);

                    Rv[idx+1] = new_pose.so3();
                    Tv[idx+1] = new_pose.translation();

                    local_map_->Clear();
                }

            }

        }

    }


    bool StateManager::TryPerformInitialization() {
        auto radarTarAryForInit = dataMagr->GetRadarTarAryForInitSafely();

        double sTimeRadar = radarTarAryForInit.front()->GetTargets().front()->GetTimestamp();
        double eTimeRadar = radarTarAryForInit.back()->GetTargets().back()->GetTimestamp();

        auto lidarDataSeq = dataMagr->ExtractLiDARDataPieceSafely(sTimeRadar, eTimeRadar);
        double eTimeLidar = lidarDataSeq.back()->GetTimestamp().second;
        auto imuDataSeq = dataMagr->ExtractIMUDataPieceSafely(sTimeRadar, eTimeLidar);

        double sTime = std::max(sTimeRadar, imuDataSeq.front()->GetTimestamp());
        double eTime = std::min(eTimeLidar, imuDataSeq.back()->GetTimestamp());

        spdlog::info(
                "'initialization': start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
                sTime, eTime, eTime - sTime
        );

        // create splines
        splines = CreateSplines(sTime, eTime);

        // init filters
        InitializeBiasFilters(sTime);

        // Calculate Relative ICP
        std::vector<Sophus::SE3d> T_B_rel(Rv.size(), Sophus::SE3d{}); // relative SE3 pose between two consecutive ***body frames*** B_{k} -> B_{k+1}, 0th element is identity
        for (size_t k = 0; k < Rv.size(); ++k) {
            Sophus::SE3d T_L_rel(Rv[k], Tv[k]);
            T_B_rel[k] = T_L_rel;
        }

        // ---------------------
        // initialize so3 spline
        // ---------------------
        spdlog::stopwatch sw;

        InitializeSO3Spline(imuDataSeq, T_B_rel, lidar_scan_times);
        spdlog::info("initialized rotation spline time elapsed: {} (s).", sw);

        // -------------------------
        // initialize gravity vector
        // -------------------------
        spdlog::stopwatch sw2;
        InitializeGravity(radarTarAryForInit, imuDataSeq);

        InitializePoseSpline(imuDataSeq, T_B_rel, lidar_scan_times);

        spdlog::info("initialize gravity and pose spline time elapsed: {} (s)", sw2);

        // observability condition is good, the gravity has been initialized
        spdlog::info(
                "initialized gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );

        // ----------------------
        // jointly refine rotation and pose splines
        // ----------------------
        spdlog::stopwatch sw3;

        auto estimator = InitializeBothSpline(imuDataSeq, T_B_rel, lidar_scan_times);

        spdlog::info("initialize both rotation and pose splines time elapsed: {} (s)", sw3);


        spdlog::info(
                "gravity vector after joint spline initialization: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );

        // ------------------
        // update bias filter
        // ------------------
        UpdateBiasFilters(estimator, eTime);

        // -----------------------
        // align states to gravity
        // -----------------------
        AlignInitializedStates();

        spdlog::info(
                "aligned gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );

        // ---------------------------------
        // marginalization in initialization
        // ---------------------------------
        spdlog::stopwatch sw4;

        MarginalizationInInit(estimator);

        spdlog::info("marginalization in initialization time elapsed: {} (s)", sw4);

        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &posSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);

        // build local map
        spdlog::stopwatch sw5;
        for (const auto &lidar_data : lidarDataSeq) {
            std::vector<Eigen::Vector3d> points_in_body;
            TransformLidarPointsToIMUFrame(lidar_data, points_in_body);
            const auto timestamps = GetTimestamps(lidar_data->GetScan());

            const auto& [frame_pair, source_pair] = Voxelize(points_in_body, timestamps);
            const auto& [frame_downsample, frame_downsample_ts] = frame_pair;
            const auto& [source, source_ts] = source_pair;

            std::vector<Eigen::Vector3d> points_deskewed;
            points_deskewed.reserve(frame_downsample.size());

            if (lidar_data->GetTimestamp().first < so3Spline.MinTime() || lidar_data->GetTimestamp().first >= so3Spline.MaxTime()) { continue; }
            if (lidar_data->GetTimestamp().first < posSpline.MinTime() || lidar_data->GetTimestamp().first >= posSpline.MaxTime()) { continue; }

            auto SO3_BtoB0 = so3Spline.Evaluate(lidar_data->GetTimestamp().first);
            auto pos_BinB0inB0 = posSpline.Evaluate(lidar_data->GetTimestamp().first);

            for(size_t i = 0; i < frame_downsample.size(); ++i) {
                double t = frame_downsample_ts[i];
                if (t < so3Spline.MinTime() || t >= so3Spline.MaxTime()) { continue; }
                if (t < posSpline.MinTime() || t >= posSpline.MaxTime()) { continue; }

                auto SO3 = so3Spline.Evaluate(t);
                auto pos = posSpline.Evaluate(t);

                Eigen::Vector3d point_deskewed = SO3_BtoB0.matrix().transpose() * (SO3.matrix() * frame_downsample[i] + pos) - SO3_BtoB0.matrix().transpose() * pos_BinB0inB0;

                points_deskewed.emplace_back(point_deskewed);
            }

            Sophus::SE3d lidar_pose(SO3_BtoB0, pos_BinB0inB0);
            local_map_->Update(points_deskewed, lidar_pose);
        }
        spdlog::info("built local map time elapsed: {} (s)", sw5);

        LOCK_TRAIL_STATUS
        TRaILStatus::StateManager::CurStatus |= TRaILStatus::StateManager::Status::HasInitialized;
        TRaILStatus::StateManager::CurStatus |= TRaILStatus::StateManager::Status::NewStateNeedToDraw;
        TRaILStatus::StateManager::CurStatus |= TRaILStatus::StateManager::Status::NewStateNeedToPublish;
        TRaILStatus::StateManager::ValidStateEndTime = eTime;

        return true;
    }

    StateManager::SplineBundleType::Ptr StateManager::CreateSplines(double sTime, double eTime) const {
        // create rotation and position splines
        auto so3SplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::SO3Spline, ns_ctraj::SplineType::So3Spline,
                sTime, eTime, configor->prior.SO3SplineKnotDist
        );
        auto posSplineInfo = ns_ctraj::SplineInfo(
                Configor::Preference::PoseSpline, ns_ctraj::SplineType::RdSpline,
                sTime, eTime, configor->prior.PoseSplineKnotDist
        );
        return SplineBundleType::Create({so3SplineInfo, posSplineInfo});
    }

    void StateManager::InitializeSO3Spline(const std::list<IMUFrame::Ptr> &imuData, const std::vector<Sophus::SE3d> &T_B_rel, const std::vector<double> &lidar_scan_times) {
        // add gyro factors and fit so3 spline
        TRaILOpt option = TRaILOpt::OPT_SO3;
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);
        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, option, configor->prior.GyroWeight);
        }

        // add lidar pose factors
        for (size_t i = 1; i < lidar_scan_times.size(); ++i) {
            spdlog::info("adding rotation constraint between lidar scans at time {:.6f} and {:.6f}", lidar_scan_times[i-1], lidar_scan_times[i]);
            estimator->AddRotationConstraint(lidar_scan_times[i-1], lidar_scan_times[i], T_B_rel[i].so3(), option, 10 * configor->prior.GyroWeight);
        }

        // make this problem fun rank
        estimator->SetParameterBlockConstant(
                splines->GetSo3Spline(Configor::Preference::SO3Spline).KnotsFront().data()
        );

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeSO3Spline':\n{}\n", sum.BriefReport());
        }

    }

    Estimator::PlaneMatches StateManager::KNNDataAssociation(const StateManager::PointsTsPair &frame_pair, std::vector<int> &sampled_indices,
                             std::size_t K,
                             double neighbor_sqdist_thresh = 1.0,
                             double plane_point_res_thresh = 0.1)
    {
        // 1. transform lidar points to world frame
        const auto& [frame_downsample, timestamps] = frame_pair;
        std::vector<Eigen::Vector3d> points_in_world;
        std::vector<Eigen::Vector3d> points_in_body;
        const auto &posSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);
        TransformLidarPointsToWorldFrame(frame_pair, points_in_world);
        TransformLidarPointsToIMUFrame(frame_pair, points_in_body);

        const std::vector<int> is_sampled = sampled_indices;

        std::fill(sampled_indices.begin(), sampled_indices.end(), 0);

        // 2. for each point, find plane correspondences from local map
        using It = std::vector<Eigen::Vector3d>::const_iterator;
        Estimator::PlaneMatches out;
        out.reserve(points_in_world.size());

        tbb::parallel_for(tbb::blocked_range<It>(points_in_world.cbegin(), points_in_world.cend()),
        [&](const tbb::blocked_range<It>& r) {
            std::vector<Estimator::PlaneMatch> local;
            local.reserve((size_t)std::distance(r.begin(), r.end()));

            for (auto it = r.begin(); it != r.end(); ++it) {
                const Eigen::Vector3d& p = *it;
                const size_t idx = static_cast<size_t>(it - points_in_world.cbegin());

                auto neighbors = local_map_->GetKClosestNeighbors(p, K);

                if (neighbors.size() < K) continue;

                const double farthest_dist = neighbors.back().second;
                if (farthest_dist * farthest_dist > neighbor_sqdist_thresh) continue;

                // 1) plane fitting via least squares
                Eigen::MatrixXd A(K, 3);
                Eigen::VectorXd b = -Eigen::VectorXd::Ones(K);
                for (size_t j = 0; j < K; ++j) {
                    const auto& pj = neighbors[j].first;
                    A(j, 0) = pj.x();
                    A(j, 1) = pj.y();
                    A(j, 2) = pj.z();
                }

                Eigen::Vector3d n_raw;
                n_raw = A.colPivHouseholderQr().solve(b);
                const double n_raw_norm = n_raw.norm();
                if (!std::isfinite(n_raw_norm) || n_raw_norm <= 1e-9) continue;

                double d = 1.0 / n_raw_norm;
                Eigen::Vector3d n = n_raw / n_raw_norm;  // n^T * A + d = 0

                // 2-b) (Check |n·p + d| <= 0.1)
                bool plane_valid = true;
                for (size_t j = 0; j < K; ++j) {
                    const auto& pj = neighbors[j].first;
                    const double rj = std::abs(n.dot(pj) + d);
                    if (!(std::isfinite(rj)) || rj > plane_point_res_thresh) {
                        plane_valid = false;
                        break;
                    }
                }
                if (!plane_valid) continue;

                // 3) based on plane model, compute the weight (score)
                const double ori_pt_dist = p.norm();

                const double pd = n.dot(p) + d;
                double s = 1.0 - 0.9 * std::abs(pd) / std::sqrt(ori_pt_dist);

                if (s > 0.9) {
                    // calculate centroid of neighbors
                    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
                    for (size_t j = 0; j < K; ++j) {
                        centroid += neighbors[j].first;
                    }
                    centroid /= static_cast<double>(K);

                    Estimator::PlaneMatch pm;
                    pm.p = points_in_body[idx];     // query point
                    pm.n = n;               // normal
                    pm.timestamp = timestamps[idx]; // timestamp
                    pm.q = centroid;        // centroid
                    pm.weight = 1/std::abs(pd);          // weight

                    // const auto coeff = posSpline.Evaluate_coeff(pm.timestamp);

                    const auto ui = posSpline.ComputeTIndex(pm.timestamp);
                    const std::size_t s = ui.second;

                    // pm.B = coeff;
                    pm.s = s;

                    if (is_sampled[idx] == 1) {
                        pm.sampled_valid = true;
                        sampled_indices[idx] = 1;
                    }
                    else {
                        pm.sampled_valid = false;
                        sampled_indices[idx] = 0;
                    }

                    local.emplace_back(std::move(pm));
                }
            }

            if (!local.empty()) {
                auto dst = out.grow_by(local.size());
                std::copy(local.begin(), local.end(), dst);
            }
        });

        return out;
    }

    Estimator::PlaneMatches StateManager::KNNDataAssociation(const StateManager::PointsTsPair &frame_pair,
                             std::size_t K,
                             double neighbor_sqdist_thresh = 1.0,
                             double plane_point_res_thresh = 0.1)
    {
        // 1. transform lidar points to world frame
        const auto& [frame_downsample, timestamps] = frame_pair;
        std::vector<Eigen::Vector3d> points_in_world;
        std::vector<Eigen::Vector3d> points_in_body;
        TransformLidarPointsToWorldFrame(frame_pair, points_in_world);
        TransformLidarPointsToIMUFrame(frame_pair, points_in_body);

        // 2. for each point, find plane correspondences from local map
        using It = std::vector<Eigen::Vector3d>::const_iterator;
        Estimator::PlaneMatches out;
        out.reserve(points_in_world.size());

        tbb::parallel_for(tbb::blocked_range<It>(points_in_world.cbegin(), points_in_world.cend()),
        [&](const tbb::blocked_range<It>& r) {
            std::vector<Estimator::PlaneMatch> local;
            local.reserve((size_t)std::distance(r.begin(), r.end()));

            for (auto it = r.begin(); it != r.end(); ++it) {
                const Eigen::Vector3d& p = *it;
                const size_t idx = static_cast<size_t>(it - points_in_world.cbegin());

                auto neighbors = local_map_->GetKClosestNeighbors(p, K);

                if (neighbors.size() < K) continue;

                const double farthest_dist = neighbors.back().second;
                if (farthest_dist * farthest_dist > neighbor_sqdist_thresh) continue;

                // 1) plane fitting via least squares
                Eigen::MatrixXd A(K, 3);
                Eigen::VectorXd b = -Eigen::VectorXd::Ones(K);
                for (size_t j = 0; j < K; ++j) {
                    const auto& pj = neighbors[j].first;
                    A(j, 0) = pj.x();
                    A(j, 1) = pj.y();
                    A(j, 2) = pj.z();
                }

                Eigen::Vector3d n_raw;
                n_raw = A.colPivHouseholderQr().solve(b);
                const double n_raw_norm = n_raw.norm();
                if (!std::isfinite(n_raw_norm) || n_raw_norm <= 1e-9) continue;

                double d = 1.0 / n_raw_norm;
                Eigen::Vector3d n = n_raw / n_raw_norm;  // n^T * A + d = 0

                // 2-b) (Check |n·p + d| <= 0.1)
                bool plane_valid = true;
                for (size_t j = 0; j < K; ++j) {
                    const auto& pj = neighbors[j].first;
                    const double rj = std::abs(n.dot(pj) + d);
                    if (!(std::isfinite(rj)) || rj > plane_point_res_thresh) {
                        plane_valid = false;
                        break;
                    }
                }
                if (!plane_valid) continue;

                // 3) based on plane model, compute the weight (score)
                const double ori_pt_dist = p.norm();

                const double pd = n.dot(p) + d;
                double s = 1.0 - 0.9 * std::abs(pd) / std::sqrt(ori_pt_dist);

                if (s > 0.9) {
                    // calculate centroid of neighbors
                    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
                    for (size_t j = 0; j < K; ++j) {
                        centroid += neighbors[j].first;
                    }
                    centroid /= static_cast<double>(K);

                    Estimator::PlaneMatch pm;
                    pm.p = points_in_body[idx];     // query point
                    pm.n = n;               // normal
                    pm.timestamp = timestamps[idx]; // timestamp
                    pm.q = centroid;        // centroid
                    pm.weight = 1/std::abs(pd);          // weight
                    pm.sampled_valid = true;
                    local.emplace_back(std::move(pm));
                }
            }

            if (!local.empty()) {
                auto dst = out.grow_by(local.size());
                std::copy(local.begin(), local.end(), dst);
            }
        });

        return out;
    }

    Estimator::PlaneMatches StateManager::KNNDataAssociation(const std::vector<Eigen::Vector3d> &points_in_world,
                             std::size_t K,
                             double neighbor_sqdist_thresh = 1.0,
                             double plane_point_res_thresh = 0.1)
    {

        // 2. for each point, find plane correspondences from local map
        using It = std::vector<Eigen::Vector3d>::const_iterator;
        Estimator::PlaneMatches out;
        out.reserve(points_in_world.size());

        tbb::parallel_for(tbb::blocked_range<It>(points_in_world.cbegin(), points_in_world.cend()),
        [&](const tbb::blocked_range<It>& r) {
            std::vector<Estimator::PlaneMatch> local;
            local.reserve((size_t)std::distance(r.begin(), r.end()));

            for (auto it = r.begin(); it != r.end(); ++it) {
                const Eigen::Vector3d& p = *it;
                const size_t idx = static_cast<size_t>(it - points_in_world.cbegin());

                auto neighbors = local_map_->GetKClosestNeighbors(p, K);

                if (neighbors.size() < K) continue;

                const double farthest_dist = neighbors.back().second;
                if (farthest_dist * farthest_dist > neighbor_sqdist_thresh) continue;

                // 1) plane fitting via least squares
                Eigen::MatrixXd A(K, 3);
                Eigen::VectorXd b = -Eigen::VectorXd::Ones(K);
                for (size_t j = 0; j < K; ++j) {
                    const auto& pj = neighbors[j].first;
                    A(j, 0) = pj.x();
                    A(j, 1) = pj.y();
                    A(j, 2) = pj.z();
                }

                Eigen::Vector3d n_raw;
                n_raw = A.colPivHouseholderQr().solve(b);
                const double n_raw_norm = n_raw.norm();
                if (!std::isfinite(n_raw_norm) || n_raw_norm <= 1e-9) continue;

                double d = 1.0 / n_raw_norm;
                Eigen::Vector3d n = n_raw / n_raw_norm;  // n^T * A + d = 0

                // 2-b) (Check |n·p + d| <= 0.1)
                bool plane_valid = true;
                for (size_t j = 0; j < K; ++j) {
                    const auto& pj = neighbors[j].first;
                    const double rj = std::abs(n.dot(pj) + d);
                    if (!(std::isfinite(rj)) || rj > plane_point_res_thresh) {
                        plane_valid = false;
                        break;
                    }
                }
                if (!plane_valid) continue;

                // 3) based on plane model, compute the weight (score)
                const double ori_pt_dist = p.norm();

                const double pd = n.dot(p) + d;
                double s = 1.0 - 0.9 * std::abs(pd) / std::sqrt(ori_pt_dist);

                if (s > 0.9) {
                    // calculate centroid of neighbors
                    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
                    for (size_t j = 0; j < K; ++j) {
                        centroid += neighbors[j].first;
                    }
                    centroid /= static_cast<double>(K);

                    Estimator::PlaneMatch pm;
                    pm.p = points_in_world[idx];     // query point
                    pm.n = n;               // normal
                    pm.timestamp = 0.0; // timestamp
                    pm.q = centroid;        // centroid
                    pm.weight = 1/std::abs(pd);          // weight
                    local.emplace_back(std::move(pm));
                }
            }

            if (!local.empty()) {
                auto dst = out.grow_by(local.size());
                std::copy(local.begin(), local.end(), dst);
            }
        });

        return out;
    }

    void StateManager::TransformLidarPointsToIMUFrame(const StateManager::PointsTsPair &frame_pair, std::vector<Eigen::Vector3d> &points_in_body) {
        // const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        // const auto &posSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);

        const auto& [points, timestamps] = frame_pair;


        if (points.size() != timestamps.size()) {
            points_in_body.clear();
            spdlog::error("points size and timestamps size are not equal in 'TransformLidarPointsToIMUFrame'!");
            return;
        }

        points_in_body.clear();
        points_in_body.reserve(points.size());

        const auto &SO3_LtoB = configor->dataStream.CalibParam.SO3_LtoB;
        const Eigen::Vector3d &POS_LinB = configor->dataStream.CalibParam.POS_LinB;

        for(size_t i = 0; i < points.size(); ++i) {

            Eigen::Vector3d point_in_body = SO3_LtoB * points[i] + POS_LinB;

            points_in_body.emplace_back(point_in_body);
        }

    }

    void StateManager::TransformLidarPointsToIMUFrame(const LidarTarget::Ptr &lidar_target, std::vector<Eigen::Vector3d> &points_in_body) {
        const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        const auto &posSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);

        const auto points = PointCloud2ToEigen(lidar_target->GetScan());
        const auto timestamps = GetTimestamps(lidar_target->GetScan());

        if (points.size() != timestamps.size()) {
            points_in_body.clear();
            spdlog::error("points size and timestamps size are not equal in 'TransformLidarPointsToIMUFrame'!");
            return;
        }

        // points_in_body.clear();
        // points_in_body.reserve(points.size());

        const auto &SO3_LtoB = configor->dataStream.CalibParam.SO3_LtoB;
        const Eigen::Vector3d &POS_LinB = configor->dataStream.CalibParam.POS_LinB;

        for(size_t i = 0; i < points.size(); ++i) {

            Eigen::Vector3d point_in_body = SO3_LtoB * points[i] + POS_LinB;

            points_in_body.emplace_back(point_in_body);
        }

    }

    void StateManager::TransformLidarPointsToWorldFrame(const StateManager::PointsTsPair &frame_pair, std::vector<Eigen::Vector3d> &points_in_world) {
        const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        const auto &posSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);

        const auto& [points, timestamps] = frame_pair;

        if (points.size() != timestamps.size()) {
            points_in_world.clear();
            spdlog::error("points size and timestamps size are not equal in 'TransformLidarPointsToWorldFrame'!");
            return;
        }

        points_in_world.clear();
        points_in_world.reserve(points.size());

        const auto &SO3_LtoB = configor->dataStream.CalibParam.SO3_LtoB;
        const Eigen::Vector3d &POS_LinB = configor->dataStream.CalibParam.POS_LinB;

        for(size_t i = 0; i < points.size(); ++i) {
            double t = timestamps[i];
            if (t < so3Spline.MinTime() || t >= so3Spline.MaxTime()) { continue; }

            auto SO3_BtoB0 = so3Spline.Evaluate(t);
            auto pos_BinB0inB0 = posSpline.Evaluate(t);

            Eigen::Vector3d point_in_world = SO3_BtoB0 * (SO3_LtoB * points[i] + POS_LinB) + pos_BinB0inB0;

            points_in_world.emplace_back(point_in_world);
        }

    }

    void StateManager::TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
        std::transform(points.cbegin(), points.cend(), points.begin(),
                    [&](const auto &point) { return T * point; });
    }

    //for point to plane
    StateManager::LinearSystem StateManager::BuildLinearSystem(const std::vector<Eigen::Vector3d> &points,
        double kernel,
        double alpha) {

        struct ResultTuple {
            Matrix6d JTJ;
            Vector6d JTr;

            ResultTuple() : JTJ(Matrix6d::Zero()), JTr(Vector6d::Zero()) {}

            ResultTuple operator+(const ResultTuple &other) const {
                ResultTuple result;
                result.JTJ = JTJ + other.JTJ;
                result.JTr = JTr + other.JTr;
                return result;
            }
        };
        const auto correspondences = KNNDataAssociation(points, 5);
        // cout<<"correspondences size: "<<correspondences.size()<<endl;
        // Point-to-Plane Jacobian and Residual
        auto compute_jacobian_and_residual_planar = [&](auto i) {
            double r_planar = (correspondences[i].p - correspondences[i].q).dot(correspondences[i].n); // residual
            Eigen::Matrix<double, 1, 6> J_planar; // Jacobian matrix
            J_planar.block<1, 3>(0, 0) = correspondences[i].n.transpose();
            J_planar.block<1, 3>(0, 3) = (correspondences[i].p.cross(correspondences[i].n)).transpose();
            return std::make_tuple(J_planar, r_planar);
        };

        double kernel_squared = kernel * kernel;
        auto compute = [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
            auto Weight = [&](double residual_squared) {
                return kernel_squared / ((kernel + residual_squared)*(kernel + residual_squared));
            };
            auto &[JTJ_private, JTr_private] = J;
            for (size_t i = r.begin(); i < r.end(); ++i) {
                if (i < correspondences.size()) { // Point-to-Plane
                    const auto &[J_planar, r_planar] = compute_jacobian_and_residual_planar(i);
                    double w_planar = Weight(r_planar * r_planar);
                    JTJ_private.noalias() += alpha * J_planar.transpose() * w_planar * J_planar;
                    JTr_private.noalias() += alpha * J_planar.transpose() * w_planar * r_planar;
                }
                // else { // Point-to-Point
                //     size_t index = i - correspondences.size();
                //     if (index < src_non_planar.size()) {
                //         const auto &[J_non_planar, r_non_planar] = compute_jacobian_and_residual_non_planar(index);
                //         const double w_non_planar = Weight(r_non_planar.squaredNorm());
                //         JTJ_private.noalias() += (1 - alpha) * J_non_planar.transpose() * w_non_planar * J_non_planar;
                //         JTr_private.noalias() += (1 - alpha) * J_non_planar.transpose() * w_non_planar * r_non_planar;
                //     }
                // }
            }
            return J;
        };


        size_t total_size = correspondences.size();
        const auto &[JTJ, JTr] = tbb::parallel_reduce(
            tbb::blocked_range<size_t>(0, total_size),
            ResultTuple(),
            compute,
            [](const ResultTuple &a, const ResultTuple &b) {
                return a + b;
            });

        return {JTJ, JTr};
    }

    Sophus::SE3d StateManager::AlignPointToPlane(const StateManager::PointsTsPair &frame_pair,
                                                const Sophus::SE3d &initial_guess,
                                                double kernel) {

        // for visualization
        std::vector<Eigen::Vector3d> final_planar_points;
        std::vector<Eigen::Vector3d> final_non_planar_points;
        final_planar_points.clear();
        final_non_planar_points.clear();

        if (local_map_->Empty()) return initial_guess;

        // std::vector<Eigen::Vector3d> source;
        // const auto& [points, timestamps] = frame_pair;
        std::vector<Eigen::Vector3d> source;
        TransformLidarPointsToIMUFrame(frame_pair, source);
        TransformPoints(initial_guess, source);

        // GenZ-ICP-loop
        Sophus::SE3d T_icp = Sophus::SE3d();
        for (int j = 0; j < 100; ++j) {
            const auto &[JTJ, JTr] = BuildLinearSystem(source, kernel, 1.0);
            const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
            const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
            TransformPoints(estimation, source);
            // Update iterations
            T_icp = estimation * T_icp;
            // Termination criteria
            if (dx.norm() < 0.0001 || j == 100 - 1) {
                break;
            }
        }

        // // Spit the final transformation
        return T_icp * initial_guess;
    }

    void StateManager::InitializeGravity(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                                         const std::list<IMUFrame::Ptr> &imuData) {
        // obtain extrinsics
        const auto &SO3_RtoB = configor->dataStream.CalibParam.SO3_RtoB;
        const Eigen::Vector3d &POS_RinB = configor->dataStream.CalibParam.POS_RinB;

        // obtain the fitted so3 spline
        const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);

        // timestamp, imu velocity computed from radar measurements
        std::vector<std::pair<double, Eigen::Vector3d>> LIN_VEL_BtoB0inB0_VEC;

        // compute the velocity of imu (with respect to the reference frame expressed in the reference frame)
        for (const auto &tarAry: radarTarAryVec) {
            double t = tarAry->GetTimestamp();

            if (t < so3Spline.MinTime() || t >= so3Spline.MaxTime()) { continue; }

            auto SO3_BtoB0 = so3Spline.Evaluate(t);
            Eigen::Vector3d ANG_VEL_BtoB0inB0 = SO3_BtoB0 * so3Spline.VelocityBody(t);

            Sophus::SO3d SO3_RtoB0 = SO3_BtoB0 * SO3_RtoB;
            Eigen::Vector3d LIN_VEL_RtoB0InB0 = tarAry->RadarVelocityFromStaticTargetArray(SO3_RtoB0);

            Eigen::Vector3d LIN_VEL_BtoB0inB0 =
                    LIN_VEL_RtoB0InB0 + Sophus::SO3d::hat(SO3_BtoB0 * POS_RinB) * ANG_VEL_BtoB0inB0;

            LIN_VEL_BtoB0inB0_VEC.emplace_back(t, LIN_VEL_BtoB0inB0);
        }
        Eigen::Vector3d mean;
        Eigen::MatrixXd var;
        Eigen::MatrixXd matrix(LIN_VEL_BtoB0inB0_VEC.size(), 3);
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);
        for (int i = 0; i < static_cast<int>(LIN_VEL_BtoB0inB0_VEC.size()) - 1; ++i) {
            int j = i + 1;
            const auto &[ti, vi] = LIN_VEL_BtoB0inB0_VEC.at(i);
            const auto &[tj, vj] = LIN_VEL_BtoB0inB0_VEC.at(j);
            matrix.row(i++) = vi;
            auto [sIter, eIter] = ExtractRange(imuData, ti, tj);
            std::vector<std::pair<double, Eigen::Vector3d>> velData;
            for (auto iter = sIter; iter != eIter; ++iter) {
                const auto &frame = *iter;
                double t = frame->GetTimestamp();
                velData.emplace_back(t, so3Spline.Evaluate(t) * frame->GetAcce());
            }
            Eigen::Vector3d velPIM = TrapIntegrationOnce(velData);
            estimator->AddVelPIMForGravityRecovery(tj - ti, vj - vi, velPIM, TRaILOpt::OPT_GRAVITY, 1.0);
        }

        if (LIN_VEL_BtoB0inB0_VEC.size() != 0) {
            mean = matrix.colwise().mean();
            var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                                (matrix.rowwise() - matrix.colwise().mean())) / static_cast<double>(LIN_VEL_BtoB0inB0_VEC.size() - 1);
        }

        if (mean.norm()<0.01 && var.diagonal().norm() < 0.01){
            auto [acceMean, acceVar] = DataManager::AcceMeanVar(imuData);
            estimator->AddStationaryGravity(acceMean);
        }

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeGravity':\n{}\n", sum.BriefReport());
        }
    }

    void StateManager::InitializePoseSpline(const std::list<IMUFrame::Ptr> &imuData, const std::vector<Sophus::SE3d> &T_B_rel, const std::vector<double> &lidar_scan_times) {
        // fit the pose spline while keeping rotation, gravity, and accelerometer bias constant
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);

        // optimization option in initialization
        TRaILOpt option = TRaILOpt::OPT_POS;
        // accelerometer factors
        for (const auto &frame: imuData) {
            estimator->AddAcceMeasurementWithConstBias(frame, option, configor->prior.AcceWeight); // let rotation as constant
        }
        // add lidar pose factors
        for (size_t i = 1; i < lidar_scan_times.size(); ++i) {
            estimator->AddPoseConstraint(lidar_scan_times[i-1], lidar_scan_times[i], T_B_rel[i].translation(), option, 1000 * configor->prior.AcceWeight); // let rotation as constant
        }

        estimator->SetParameterBlockConstant(
                splines->GetRdSpline(Configor::Preference::PoseSpline).KnotsFront().data()
        );

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(4, Configor::Preference::DEBUG_MODE, false));

    }

    Estimator::Ptr StateManager::InitializeBothSpline(const std::list<IMUFrame::Ptr> &imuData,
                                                      const std::vector<Sophus::SE3d> &T_B_rel,
                                                      const std::vector<double> &lidar_scan_times) {

        // jointly fit the rotation and pose splines
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);

        // optimization option in initialization
        TRaILOpt option = TRaILOpt::OPT_SO3 | TRaILOpt::OPT_POS;

        // acce and gyro factors
        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, option, configor->prior.GyroWeight);
            estimator->AddAcceMeasurementWithConstBias(frame, option, configor->prior.AcceWeight);
        }

        for (size_t i = 1; i < lidar_scan_times.size(); ++i) {
            estimator->AddPoseConstraint(lidar_scan_times[i-1], lidar_scan_times[i], T_B_rel[i].translation(), option, 1000 * configor->prior.AcceWeight); // let rotation as constant
            estimator->AddRotationConstraint(lidar_scan_times[i-1], lidar_scan_times[i], T_B_rel[i].so3(), option, 10 * configor->prior.GyroWeight);
        }

        // add a tail constraint to handle the poor observability of the last knot
        auto poseTailIdVec = estimator->AddPoseSplineTailConstraint(option, 1000.0);
        auto so3TailIdVec = estimator->AddSo3SplineTailConstraint(option, 1000.0);

        // make this problem full rank
        estimator->SetParameterBlockConstant(
                splines->GetSo3Spline(Configor::Preference::SO3Spline).KnotsFront().data()
        );

        estimator->SetParameterBlockConstant(
                splines->GetRdSpline(Configor::Preference::PoseSpline).KnotsFront().data()
        );

        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(4, Configor::Preference::DEBUG_MODE, false));


        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("here is the summary of 'InitializeBothSpline':\n{}\n", sum.BriefReport());
        }

        for (const auto &id: poseTailIdVec) { estimator->RemoveResidualBlock(id); }
        for (const auto &id: so3TailIdVec) { estimator->RemoveResidualBlock(id); }

        return estimator;
    }

    void StateManager::InitializeBiasFilters(double sTime) {
        {
            // -----------------
            // acceleration bias
            // -----------------
            BiasFilter::StatePack initState(
                    sTime, *ba, Eigen::Vector3d::Ones() * 0.0001 * 0.0001
            );
            baFilter = BiasFilter::Create(initState, configor->prior.AcceBiasRandomWalk);
            spdlog::info("initial state of ba filter: {}", TRAIL_TO_STR(baFilter->GetCurState()));

        }

        {
            // --------------
            // gyroscope bias
            // --------------
            BiasFilter::StatePack initState(
                    sTime, *bg, Eigen::Vector3d::Ones() * 0.00001 * 0.00001
            );
            bgFilter = BiasFilter::Create(initState, configor->prior.GyroBiasRandomWalk);
            spdlog::info("initial state of bg filter: {}", TRAIL_TO_STR(bgFilter->GetCurState()));
        }

    }

    void StateManager::AlignInitializedStates() {
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &poseSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);
        // current gravity, velocities, and rotations are expressed in the reference frame
        // align them to the world frame whose negative z axis is aligned with the gravity vector
        spdlog::info(
                "aligned gravity vector1: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
                (*gravity)(0), (*gravity)(1), (*gravity)(2)
        );
    }

    void StateManager::MarginalizationInInit(const Estimator::Ptr &estimator) {
        // perform marginalization
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &poseSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);
        int so3KnotSize = static_cast<int>(so3Spline.GetKnots().size());
        int poseKnotSize = static_cast<int>(poseSpline.GetKnots().size());

        std::set<double *> margParBlocks{gravity->data()};
        for (int i = 0; i < so3KnotSize; ++i) {
            if (i <= so3KnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(so3Spline.GetKnot(i).data());
            } else {
                lastKeepSo3KnotAdd.insert({i, so3Spline.GetKnot(i).data()});
            }
        }
        for (int i = 0; i < poseKnotSize; ++i) {
            if (i <= poseKnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(poseSpline.GetKnot(i).data());
            } else {
                lastKeepPosKnotAdd.insert({i, poseSpline.GetKnot(i).data()});
            }
        }

        if (Configor::Preference::DEBUG_MODE) {
            spdlog::info("rotation knots: {}, pose knots: {}", so3KnotSize, poseKnotSize);
            spdlog::info(
                    "marginalize rotation knots from 0-th to {}-th", so3KnotSize - Configor::Prior::SplineOrder - 1
            );
            spdlog::info(
                    "marginalize pose knots from 0-th to {}-th", poseKnotSize - Configor::Prior::SplineOrder - 1
            );
        }
        margInfo = ns_ctraj::MarginalizationInfo::Create(estimator.get(), margParBlocks, {}, 2);
        spdlog::info("marginalization in initialization is done.");
    }

    // --------------------------------
    //    top: initialization
    // --------------------------------
    // bottom: incremental optimization
    // --------------------------------

    void StateManager::PreOptimization(const std::list<IMUFrame::Ptr> &imuData, double stime, double etime) {
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);

        // --------------------------
        // optimize rotation b-spline
        // --------------------------

        for (const auto &frame: imuData) {
            estimator->AddGyroMeasurementWithConstBias(frame, TRaILOpt::OPT_SO3, configor->prior.GyroWeight);
        }
        // maintain observability of the last few control points
        estimator->AddSo3SplineTailConstraint(TRaILOpt::OPT_SO3, 1000.0);

        double dt = 0.05;
        for (double t=stime; t<etime-0.05; t+=0.01){
            estimator->AddRotPrior(t, dt, 1000);
        }

        // solving
        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

    }

    bool StateManager::IncrementalOptimization(const TRaILStatus::StatusPack &status) {
        spdlog::stopwatch sw;
        // set start time as the end time of last optimization
        const double sTime = status.ValidStateEndTime, eTime = dataMagr->GetEndTimeSafely();

        if (eTime - sTime < 1.0 / configor->preference.IncrementalOptRate) { return false; }

        auto imuData = dataMagr->ExtractIMUDataPieceSafely(sTime, eTime);
        auto radarData = dataMagr->ExtractRadarDataPieceSafely(sTime, eTime);
        auto lidarData = dataMagr->ExtractLiDARDataPieceSafely(sTime, eTime+0.001);

        if(lidarData.size()==0){
            spdlog::info("no lidar data for incremental optimization");
            return false;
        }

        if(radarData.size()==0 && lidarData.size()==0 && imuData.size()==0){
            LOCK_TRAIL_STATUS
            spdlog::info("no data for incremental optimization, should quit.");
            TRaILStatus::StateManager::CurStatus |= TRaILStatus::StateManager::Status::ShouldQuit;
            return false;
        }

        // --------------
        // quit condition
        // --------------
        const double dt = eTime - sTime;
        const double imuFreq = static_cast<double>(imuData.size()) / dt;
        const double radarFreq = static_cast<double>(radarData.size()) / dt;

        if (imuFreq < 50 || radarFreq < 10) {
            LOCK_TRAIL_STATUS
            spdlog::info("insufficient imu or radar frequency for incremental optimization, should quit.");
            return false;
        }

        LOCK_STATES
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &posSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);

        // --------------
        // extend splines
        // --------------
        int so3KnotOldSize = static_cast<int>(so3Spline.GetKnots().size());
        int posKnotOldSize = static_cast<int>(posSpline.GetKnots().size());

        // --------------------
        // linear extrapolation
        // --------------------

        LinearExtendKnotTo(so3Spline, eTime + 1E-6);
        LinearExtendKnotTo(posSpline, eTime + 1E-6);

        // obtain the updated address
        std::map<double *, double *> updatedLastKeepKnotAdd;
        for (const auto &[idx, add]: lastKeepSo3KnotAdd) {
            updatedLastKeepKnotAdd.insert({add, so3Spline.GetKnot(idx).data()});
        }
        for (const auto &[idx, add]: lastKeepPosKnotAdd) {
            updatedLastKeepKnotAdd.insert({add, posSpline.GetKnot(idx).data()});
        }
        margInfo->ShiftKeepParBlockAddress(updatedLastKeepKnotAdd);

        if (Configor::Preference::DEBUG_MODE) {
            int parShiftCount = 0;
            for (const auto &[oldAdd, newAdd]: updatedLastKeepKnotAdd) { if (oldAdd != newAdd) { ++parShiftCount; }}
            spdlog::error(
                    "size of 'updatedLastKeepKnotAdd': {}, count of shifted parameter blocks: {}.",
                    updatedLastKeepKnotAdd.size(), parShiftCount
            );
        }


        // **********************************
        // 0. pre-optimization of rotation spline
        // **********************************

        PreOptimization(imuData, sTime, eTime);

        // ----------------------------
        // 1. Initial Guess Using RIO
        // ----------------------------
        auto estimator = Estimator::Create(configor, splines, gravity, ba, bg);

        //  marginalization factor
        ns_ctraj::MarginalizationFactor::AddToProblem(estimator.get(), margInfo, 1.0);

        TRaILOpt option = TRaILOpt::OPT_SO3 | TRaILOpt::OPT_POS;
        for (auto iter1 = imuData.cbegin(); iter1 != imuData.cend(); ++iter1){
            estimator->AddGyroMeasurementWithConstBias(*iter1, option, configor->prior.GyroWeight);
            estimator->AddAcceMeasurementWithConstBias(*iter1, option, configor->prior.AcceWeight);
        }

        auto bgPrioriId = estimator->AddGyroBiasPriori(bgFilter->Prediction(eTime), option);
        auto baPrioriId = estimator->AddAcceBiasPriori(baFilter->Prediction(eTime), option);

        // radar factors
        for (const auto &tar: radarData) {
            estimator->AddRadarMeasurement(tar, option, SO3_RefToW, configor->prior.RadarWeight);
        }

        //**radar ego vel */
        Eigen::VectorXd lVec(radarData.size());
        Eigen::MatrixXd BMat(radarData.size(), 3);
        int idx_r = 0;
        for (const auto &tar: radarData) {
            lVec(idx_r) = tar->GetRadialVelocity() * tar->GetTargetXYZ().norm();
            BMat.block<1, 3>(idx_r, 0) = -tar->GetTargetXYZ().transpose();
            idx_r++;
        }
        Eigen::Vector3d xVec = (BMat.transpose() * BMat).inverse() * BMat.transpose() * lVec;
        if(xVec.norm() < 0.01){
            spdlog::warn("unreasonable radar estimated velocity, norm: {:.6f}", xVec.norm());
        }

        auto so3TailIdVec = estimator->AddSo3SplineTailConstraint(option, 1000.0);
        auto poseTailIdVec = estimator->AddPoseSplineTailConstraint(option, 1000.0);
        auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

        // *** radar point cloud filtering using velocity ***
        std::list<RadarTarget::Ptr> inlier_radarData;
        size_t inlier_count = 0;
        size_t outlier_count = 0;

        auto ref_radar_time = sTime - 0.1;
        Eigen::Vector3d LIN_VEL = Eigen::Vector3d::Zero();

        if (ref_radar_time < so3Spline.MinTime() || ref_radar_time >= so3Spline.MaxTime()) {  }
        else{

            auto SO3_BtoB0 = so3Spline.Evaluate(ref_radar_time);
            Eigen::Vector3d ANG_VEL_BtoB0inB = so3Spline.VelocityBody(ref_radar_time);

            Eigen::Vector3d &POS_RinB = configor->dataStream.CalibParam.POS_RinB;

            Eigen::Vector3d LIN_VEL_BtoB0inB = //radar frame
                    SO3_BtoB0.matrix().transpose() * posSpline.Velocity(ref_radar_time) + Sophus::SO3d::hat(ANG_VEL_BtoB0inB) * POS_RinB;
            LIN_VEL = configor->dataStream.CalibParam.SO3_RtoB.matrix().transpose() * LIN_VEL_BtoB0inB;

        }

        for (const auto &tar: radarData) {
            Eigen::Vector3d u = tar->GetTargetXYZ() / tar->GetTargetXYZ().norm();
            double estimated_radar_radial_vel = u.transpose() * LIN_VEL;
            double diff = std::abs(tar->GetRadialVelocity() + estimated_radar_radial_vel);
            if(diff > 0.3 || tar->GetRadialVelocity() < -1.4 || tar->GetRadialVelocity() > 0.3){ //heuristic threshold, can be tuned
                outlier_count++;
            }
            else{
                inlier_count++;
                inlier_radarData.push_back(tar);
            }
        }

        if(xVec.norm() < 0.01){
            inlier_radarData.clear();
            inlier_radarData = radarData;
            spdlog::warn("radar point cloud filtering skipped, inlier count < outlier count or unreasonable radar estimated velocity.");
        }

        // ------------------------
        // 2. LiDAR ICP Refinement
        // ------------------------
        auto points = PointCloud2ToEigen(lidarData.front()->GetScan());
        auto timestamps = GetTimestamps(lidarData.front()->GetScan());


        // RMS Sampling
        spdlog::stopwatch sw_rms;
        rms::RMS rms(10, 0.008f, 0.4f, 0.4f);

        const auto& [frame_pair, source_pair] = Voxelize(points, timestamps);
        const auto& [frame_downsample, frame_downsample_ts] = frame_pair; // for local map
        const auto& [source, source_ts] = source_pair; // for data association

        auto points_rms     = frame_downsample;
        auto timestamps_rms = frame_downsample_ts;
        auto indices_rms = rms.sample(points_rms, timestamps_rms);
        auto rms_pair = StateManager::PointsTsPair{std::move(points_rms), std::move(timestamps_rms)};

        Eigen::MatrixXf degenerate_basis(0, 3);
        int degenerate_dim = 0;
        float lambda_min = 0.0f;
        float lambda_mid = 0.0f;
        float lambda_max = 0.0f;
        float kappa = std::numeric_limits<float>::infinity();
        const float kappa_thresh = 5.0f;
        const float eps = 1e-12f;
        double sphericity = 0.0;

        spdlog::stopwatch sw_reweight;

        {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(rms.Htt);
            if (es.info() == Eigen::Success) {
                // eigenvalues are ascending: λ0 ≤ λ1 ≤ λ2
                Eigen::Vector3f eval = es.eigenvalues();
                Eigen::Matrix3f evec = es.eigenvectors();

                lambda_min = eval(0);
                lambda_mid = eval(1);
                lambda_max = eval(2);

                // condition number calculation
                if (lambda_max < eps) {
                // If Htt is almost zero, it means there is almost no information (condition number is unstable by definition)
                    kappa = std::numeric_limits<float>::infinity();
                } else {
                    // Very small λmin is clipped to eps (or can be left as inf)
                    float denom = std::max(lambda_min, eps);
                    kappa = lambda_max / denom;

                    // Degenerate subspace: eigenvectors with small eigenvalues vs. lambda_max.
                    Eigen::Matrix<float, 2, 3> degenerate_basis_tmp = Eigen::Matrix<float, 2, 3>::Zero();
                    const float ratio_thresh = 1.0f / kappa_thresh;
                    if (lambda_min / lambda_max < ratio_thresh) {
                        degenerate_basis_tmp.row(degenerate_dim++) = evec.col(0).transpose();
                    }
                    if (lambda_mid / lambda_max < ratio_thresh) {
                        degenerate_basis_tmp.row(degenerate_dim++) = evec.col(1).transpose();
                    }
                    degenerate_basis = degenerate_basis_tmp.topRows(degenerate_dim);
                }

                sphericity =
                    std::cbrt(std::max(0.0f, lambda_min * lambda_mid * lambda_max)) /
                    ((lambda_min + lambda_mid + lambda_max) / 3.0 + eps);

            } else {
                kappa = std::numeric_limits<float>::infinity();
            }
        }

        latest_lidar_kappa_ = static_cast<double>(kappa);
        latest_degenerate_dim_ = degenerate_dim;

        spdlog::info("RMS time elapsed: {} (s)", sw_rms);


        //-----------------------------------------
        // Adaptive Weight Calculation per Radar measurement
        //-----------------------------------------
        const auto &SO3_RtoB = configor->dataStream.CalibParam.SO3_RtoB;
        const auto &SO3_LtoB = configor->dataStream.CalibParam.SO3_LtoB;
        std::vector<double> radar_weight(inlier_radarData.size(), 1.0);
        radar_weight.reserve(inlier_radarData.size());

        double eps_w        = 0.20;   // ε
        double w_max        = 3.00;   // w_max
        double alpha        = 20.0;   // α
        double eta          = 2.2;    // η


        //Build Radar Sensitivity Weight
        if (kappa > kappa_thresh && degenerate_dim > 0 && inlier_radarData.size() > 0) {

            const Eigen::Matrix3d R_LtoR = SO3_RtoB.matrix().transpose() * SO3_LtoB.matrix();
            const Eigen::MatrixXd degenerate_basis_radar = degenerate_basis.cast<double>() * R_LtoR.transpose();
            int i = 0;

            Eigen::MatrixXd Z(degenerate_dim, inlier_radarData.size());
            Z.setZero();
            buildRadarSensitivityMatrix(degenerate_basis_radar, inlier_radarData, Z);

            // Radar Degeneracy Subspace Observability Check
            bool degenerate_observable = isRadarSubspaceObservable(Z);
            if(degenerate_observable){
                reweightingRadarMeasurement(radar_weight, Z, eta);

                // (2) Build H_Ld (LiDAR Hessian projected to degenerate subspace)
                // degenerate_basis: (deg_dim x 3) float -> double
                Eigen::MatrixXd B = degenerate_basis.cast<double>();                 // m x 3
                Eigen::MatrixXd Htt_d = rms.Htt.cast<double>();                      // 3 x 3
                Eigen::MatrixXd H_Ld = B * Htt_d * B.transpose();                    // m x m

                // (3) Build H_Rd (Radar Hessian inside degenerate subspace)
                Eigen::VectorXd wv = Eigen::Map<const Eigen::VectorXd>(
                    radar_weight.data(), static_cast<int>(radar_weight.size())
                );                                                                    // N x 1
                Eigen::MatrixXd H_Rd = Z * wv.asDiagonal() * Z.transpose();          // m x m

                // Optional: tiny regularization for numerical stability
                const double reg = 1e-9;
                H_Ld.diagonal().array() += reg;
                H_Rd.diagonal().array() += reg;

                // Gamma schedule: gamma = 1 / max(S(N), S_min)
                const double sph = static_cast<double>(sphericity);
                const double sph_min = 1e-3;
                const double gamma = 1.0 / std::max(sph, sph_min);

                // (4) Final combined degenerate-subspace Hessian (for diagnostics / design)
                Eigen::MatrixXd H_d = H_Ld + gamma * H_Rd;

                // Print diagnostics
                if (degenerate_dim == 1) {
                    // 1x1: eigenvalue = (0,0)
                    double lmin_b = H_Ld(0,0);
                    double lmax_b = H_Ld(0,0);
                    double lmin_a = H_d (0,0);
                    double lmax_a = H_d (0,0);

                } else {
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es_before(H_Ld);
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es_after (H_d);

                    if (es_before.info() == Eigen::Success && es_after.info() == Eigen::Success) {
                        const auto& ev_b = es_before.eigenvalues();  // ascending
                        const auto& ev_a = es_after.eigenvalues();   // ascending

                        double lmin_b = ev_b(0);
                        double lmax_b = ev_b(degenerate_dim - 1);
                        double lmin_a = ev_a(0);
                        double lmax_a = ev_a(degenerate_dim - 1);
                    }
                }
                // (5) Apply gamma to the actual radar residual weights
                for (double &w : radar_weight) {
                    w *= gamma;
                }

                if(xVec.norm() < 0.01){ //zero vel detected
                    for (double &w : radar_weight) {
                        w *= 10;
                    }

                }
            }
        }

        std::vector<int> sample_idx(frame_downsample.size(), 0);
        sample_idx.reserve(frame_downsample.size());
        for (int j = 0; j < static_cast<int>(indices_rms.size()); ++j) {
            const auto idx = indices_rms[j];
            if (idx < sample_idx.size()) sample_idx[idx] = 1; // 0 : not sampled, 1 : sampled
        }

        // point wise point-to-plane residuals
        spdlog::info("----------------- ICP optimization ----------------");

        // ----------------------------------
        // 3. Re-Estimation Using pose prior
        //-----------------------------------
        auto re_estimator = Estimator::Create(configor, splines, gravity, ba, bg);
        auto init_estimator = Estimator::Create(configor, splines, gravity, ba, bg);
        option = TRaILOpt::OPT_SO3 | TRaILOpt::OPT_POS;
        spdlog::stopwatch sw_da;

        //----------------
        //4. Doppler Tikhonov ICP
        // This part help ICP to converge when there is large initial misalignment
        // It add Doppler velocity residual to the cost function to constrain the trust region
        //----------------

        for(size_t iter =0; iter < 2; ++iter){

            // 1 . transform point cloud to the body frame using current pose spline & Extract new correspondences
            spdlog::stopwatch sw_iter;
            init_estimator = Estimator::Create(configor, splines, gravity, ba, bg);
            const auto correspondences = KNNDataAssociation(rms_pair, 5); //source : LiDAR frame, correspondences : IMU gravity align frame
            ns_ctraj::MarginalizationFactor::AddToProblem(init_estimator.get(), margInfo, 1.0);
            for (auto iter1 = imuData.cbegin(); iter1 != imuData.cend(); ++iter1){
                init_estimator->AddGyroMeasurementWithConstBias(*iter1, option, configor->prior.GyroWeight);
                init_estimator->AddAcceMeasurementWithConstBias(*iter1, option, configor->prior.AcceWeight);
            }
            baPrioriId = init_estimator->AddAcceBiasPriori(baFilter->Prediction(eTime), option);
            bgPrioriId = init_estimator->AddGyroBiasPriori(bgFilter->Prediction(eTime), option);

            size_t i = 0;
            for (const auto &tar: inlier_radarData) {
                init_estimator->AddRadarMeasurement(tar, option, SO3_RefToW, configor->prior.RadarWeight * radar_weight[i]);
                i++;
            }

            for(const auto &match: correspondences){
                init_estimator->AddLiDARPointToPlaneFactor(match, configor->prior.LiDARWeight, option); // query point need to be body frame
            }

            so3TailIdVec = init_estimator->AddSo3SplineTailConstraint(option, 1000.0);
            poseTailIdVec = init_estimator->AddPoseSplineTailConstraint(option, 1000.0);

            // Keep the optimized velocity close to its initial estimate along
            // directions that are weakly constrained by LiDAR.
            const double mu = 1000;
            if (degenerate_dim > 0 && inlier_radarData.size() > 0) {
                for (const auto &tar: inlier_radarData) {
                    auto time = tar->GetTimestamp();

                    if (time < so3Spline.MinTime() || time >= so3Spline.MaxTime()) { continue; }

                    auto SO3_BtoB0 = so3Spline.Evaluate(time);
                    Eigen::Vector3d ANG_VEL_BtoB0inB = so3Spline.VelocityBody(time);

                    Eigen::Vector3d &POS_RinB = configor->dataStream.CalibParam.POS_RinB;

                    Eigen::Vector3d LIN_VEL_BtoB0inB =
                            SO3_BtoB0.matrix().transpose() * posSpline.Velocity(time) + Sophus::SO3d::hat(ANG_VEL_BtoB0inB) * POS_RinB;

                    const Eigen::MatrixXd degenerate_basis_radar = degenerate_basis.cast<double>() * SO3_LtoB.matrix().transpose();

                    init_estimator->AddRadarProxMeasurement(tar, LIN_VEL_BtoB0inB, degenerate_basis_radar, option, mu);

                }
            }

            auto sum_icp = init_estimator->Solve(Estimator::DefaultSolverOptions_init(1, Configor::Preference::DEBUG_MODE, false));

            spdlog::info("Here is the summary of init estimation:\n{}\n", sum_icp.BriefReport());

            init_estimator->RemoveResidualBlock(baPrioriId);
            init_estimator->RemoveResidualBlock(bgPrioriId);
            // remove linear extrapolation factor as they are not come from real-sensor measurements
            for (const auto &id: so3TailIdVec) { init_estimator->RemoveResidualBlock(id); }
            for (const auto &id: poseTailIdVec) { init_estimator->RemoveResidualBlock(id); }

            spdlog::info("iteration time elapsed: {} (s)", sw_iter);
        }



        for(size_t iter =0; iter < ICP_MAX_ITER; ++iter){

            // 1 . transform point cloud to the body frame using current pose spline & Extract new correspondences
            spdlog::stopwatch sw_iter;
            re_estimator = Estimator::Create(configor, splines, gravity, ba, bg);
            const auto correspondences = KNNDataAssociation(rms_pair, 5); //source : LiDAR frame, correspondences : IMU gravity align frame
            ns_ctraj::MarginalizationFactor::AddToProblem(re_estimator.get(), margInfo, 1.0);

            for (auto iter1 = imuData.cbegin(); iter1 != imuData.cend(); ++iter1){
                re_estimator->AddGyroMeasurementWithConstBias(*iter1, option, configor->prior.GyroWeight);
                re_estimator->AddAcceMeasurementWithConstBias(*iter1, option, configor->prior.AcceWeight);
            }
            baPrioriId = re_estimator->AddAcceBiasPriori(baFilter->Prediction(eTime), option);
            bgPrioriId = re_estimator->AddGyroBiasPriori(bgFilter->Prediction(eTime), option);

            size_t i = 0;
            for (const auto &tar: inlier_radarData) {
                re_estimator->AddRadarMeasurement(tar, option, SO3_RefToW, configor->prior.RadarWeight* radar_weight[i]);
                i++;
            }


            for(const auto &match: correspondences){
                re_estimator->AddLiDARPointToPlaneFactor(match, configor->prior.LiDARWeight, option); // query point need to be body frame
            }

            so3TailIdVec = re_estimator->AddSo3SplineTailConstraint(option, 1000.0);
            poseTailIdVec = re_estimator->AddPoseSplineTailConstraint(option, 1000.0);

            auto sum_icp = re_estimator->Solve(Estimator::DefaultSolverOptions_ICP(1, Configor::Preference::DEBUG_MODE, false));

            spdlog::info("Final here is the summary of estimation:\n{}\n", sum_icp.BriefReport());
            if (sum_icp.termination_type == ceres::CONVERGENCE){
                icp_iter_cnt_ = iter + 1;
                success_cnt_++;

                UpdateBiasFilters(re_estimator, eTime);
                re_estimator->RemoveResidualBlock(baPrioriId);
                re_estimator->RemoveResidualBlock(bgPrioriId);
                // remove linear extrapolation factor as they are not come from real-sensor measurements
                for (const auto &id: so3TailIdVec) { re_estimator->RemoveResidualBlock(id); }
                for (const auto &id: poseTailIdVec) { re_estimator->RemoveResidualBlock(id); }
                break;
            }
            if (iter == ICP_MAX_ITER -1){
                icp_iter_cnt_ = iter + 1;
                fail_cnt_++;
            }
            spdlog::info("iteration time elapsed: {} (s)", sw_iter);
            re_estimator->RemoveResidualBlock(baPrioriId);
            re_estimator->RemoveResidualBlock(bgPrioriId);
            // remove linear extrapolation factor as they are not come from real-sensor measurements
            for (const auto &id: so3TailIdVec) { re_estimator->RemoveResidualBlock(id); }
            for (const auto &id: poseTailIdVec) { re_estimator->RemoveResidualBlock(id); }
        }

        spdlog::info("ICP iterations : {}, success count : {}, fail count : {}", icp_iter_cnt_, success_cnt_, fail_cnt_);

        spdlog::info("Final estimation time elapsed: {} (s)", sw_da);

        // ------------------
        // build local map
        // ------------------
        spdlog::info("--------------- start of building local map  ----------------");
        spdlog::stopwatch sw_2;

        std::vector<Eigen::Vector3d> map_points_bodyframe;
        std::vector<Eigen::Vector3d> map_points_deskewed;
        map_points_bodyframe.reserve(frame_downsample.size());
        map_points_deskewed.reserve(frame_downsample.size());
        TransformLidarPointsToIMUFrame(frame_pair, map_points_bodyframe);

        auto timestamp_start = lidarData.front()->GetTimestamp().first;
        auto lidar_rot = so3Spline.Evaluate(timestamp_start);
        auto lidar_pos = posSpline.Evaluate(timestamp_start);

        for(size_t i = 0; i < map_points_bodyframe.size(); ++i) {
            double t = frame_downsample_ts[i];

            if (t < so3Spline.MinTime() || t >= so3Spline.MaxTime()) { continue; }
            if (t < posSpline.MinTime() || t >= posSpline.MaxTime()) { continue; }

            auto SO3 = so3Spline.Evaluate(t);
            auto pos = posSpline.Evaluate(t);

            Eigen::Vector3d point_deskewed = lidar_rot.matrix().transpose() * (SO3.matrix() * map_points_bodyframe[i] + pos) - lidar_rot.matrix().transpose() * lidar_pos;

            map_points_deskewed.emplace_back(point_deskewed);
        }
        spdlog::info("deskewed points size : {}", map_points_deskewed.size());

        Sophus::SE3d lidar_pose(lidar_rot, lidar_pos);
        local_map_->Update(map_points_deskewed, lidar_pose); //frame downsample must be in body frame

        spdlog::info("built local map time elapsed: {} (s)", sw_2);
        spdlog::info("--------------- end of building local map  ----------------");

        // --------------------
        // 4. Marginalization
        // --------------------
        int so3KnotSize = static_cast<int>(so3Spline.GetKnots().size());
        int poseKnotSize = static_cast<int>(posSpline.GetKnots().size());

        std::set<double *> margParBlocks{gravity->data()};
        lastKeepSo3KnotAdd.clear(), lastKeepPosKnotAdd.clear();

        for (int i = std::max(0, so3KnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < so3KnotSize; ++i) {
            if (i <= so3KnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(so3Spline.GetKnot(i).data());
            } else {
                lastKeepSo3KnotAdd.insert({i, so3Spline.GetKnot(i).data()});
            }
        }
        for (int i = std::max(0, posKnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < poseKnotSize; ++i) {
            if (i <= poseKnotSize - 2 * Configor::Prior::SplineOrder) {
                margParBlocks.insert(posSpline.GetKnot(i).data());
            } else {
                lastKeepPosKnotAdd.insert({i, posSpline.GetKnot(i).data()});
            }
        }

        spdlog::stopwatch sw_marg;

        margInfo = ns_ctraj::MarginalizationInfo::Create(re_estimator.get(), margParBlocks, {}, 1);

        spdlog::info("marg time elapsed: {} (s)", sw_marg);

        LOCK_TRAIL_STATUS
        TRaILStatus::StateManager::CurStatus |= TRaILStatus::StateManager::Status::NewStateNeedToDraw;
        TRaILStatus::StateManager::CurStatus |= TRaILStatus::StateManager::Status::NewStateNeedToPublish;
        TRaILStatus::StateManager::ValidStateEndTime = eTime;
        spdlog::info("Incremental Optimization time elapsed: {} (s)", sw);

        return true;
    }

    std::optional<StateManager::StatePack> StateManager::GetStatePackSafely(double t) const {
        LOCK_STATES
        if (splines == nullptr) { return {}; }

        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        if (!splines->TimeInRange(t, so3Spline)) { return {}; }

        auto &posSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);
        if (!splines->TimeInRange(t, posSpline)) { return {}; }

        StatePack pack;
        pack.timestamp = t;
        pack.ba = *ba;
        pack.bg = *bg;
        pack.gravity = *gravity;
        pack.lidar_kappa = latest_lidar_kappa_;
        pack.degenerate_dim = latest_degenerate_dim_;
        pack.SO3_CurToRef = so3Spline.Evaluate(t);
        pack.POS_CurToRef = posSpline.Evaluate(t);
        pack.LIN_VEL_CurToRefInCur = pack.SO3_CurToRef.inverse() * posSpline.Velocity(t);

        return pack;
    }

    std::vector<std::optional<StateManager::StatePack>>
    StateManager::GetStatePackSafely(const std::vector<double> &times) const {
        std::vector<std::optional<StateManager::StatePack>> packs(times.size());
        LOCK_STATES
        if (splines == nullptr) { return packs; }
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        auto &posSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);

        for (int i = 0; i < static_cast<int>(times.size()); ++i) {
            double t = times.at(i);
            if (!splines->TimeInRange(t, so3Spline) || !splines->TimeInRange(t, posSpline)) {
                packs.at(i) = {};
            } else {
                StatePack pack;
                pack.timestamp = t;
                pack.ba = *ba;
                pack.bg = *bg;
                pack.gravity = *gravity;
                pack.lidar_kappa = latest_lidar_kappa_;
                pack.degenerate_dim = latest_degenerate_dim_;
                pack.SO3_CurToRef = so3Spline.Evaluate(t);
                pack.POS_CurToRef = posSpline.Evaluate(t);
                pack.LIN_VEL_CurToRefInCur = pack.SO3_CurToRef.inverse() * posSpline.Velocity(t);

                packs.at(i) = pack;
            }
        }
        return packs;
    }

    const StateManager::SplineBundleType::Ptr &StateManager::GetSplines() const {
        return splines;
    }

    void StateManager::UpdateBiasFilters(const Estimator::Ptr &estimator, double eTime) {
        auto baCovOpt = ObtainVarMatFromEstimator<3, 3>({ba->data(), ba->data()}, estimator);
        if (baCovOpt != std::nullopt) {
            Eigen::Vector3d baCov = baCovOpt->diagonal();
            baFilter->UpdateByEstimator(BiasFilter::StatePack(eTime, *ba, baCov));
            spdlog::info("current state of ba filter: {}", TRAIL_TO_STR(baFilter->GetCurState()));
        }

        auto bgCovOpt = ObtainVarMatFromEstimator<3, 3>({bg->data(), bg->data()}, estimator);
        if (bgCovOpt != std::nullopt) {
            Eigen::Vector3d bgCov = bgCovOpt->diagonal();
            bgFilter->UpdateByEstimator(BiasFilter::StatePack(eTime, *bg, bgCov));
            spdlog::info("current state of bg filter: {}", TRAIL_TO_STR(bgFilter->GetCurState()));
        }

    }

    const BiasFilter::Ptr &StateManager::GetBaFilter() const {
        return baFilter;
    }

    const BiasFilter::Ptr &StateManager::GetBgFilter() const {
        return bgFilter;
    }

    void StateManager::LinearExtendKnotTo(SplineBundleType::RdSplineType &spline, double t) {
        Eigen::Vector3d delta = spline.GetKnots().back() - spline.GetKnots().at(spline.GetKnots().size() - 2);
        while ((spline.GetKnots().size() < SplineBundleType::N) || (spline.MaxTime() < t)) {
            spline.KnotsPushBack(spline.GetKnots().back() + delta);
        }
    }

    void StateManager::LinearExtendKnotTo(SplineBundleType::So3SplineType &spline, double t) {
        Sophus::SO3d delta =
                spline.GetKnots().at(spline.GetKnots().size() - 2).inverse() * spline.GetKnots().back();
        while ((spline.GetKnots().size() < SplineBundleType::N) || (spline.MaxTime() < t)) {
            spline.KnotsPushBack(spline.GetKnots().back() * delta);
        }
    }

    StateManager::StatePack::StatePack(double timestamp, const Sophus::SO3d &so3CurToRef, Eigen::Vector3d posCurToRef,
                                       Eigen::Vector3d linVelCurToRefInCur, Eigen::Vector3d gravity,
                                       Eigen::Vector3d ba, Eigen::Vector3d bg)
            : timestamp(timestamp), SO3_CurToRef(so3CurToRef), POS_CurToRef(std::move(posCurToRef)),
              LIN_VEL_CurToRefInCur(std::move(linVelCurToRefInCur)), gravity(std::move(gravity)), ba(std::move(ba)),
              bg(std::move(bg)) {}

    Eigen::Vector3d StateManager::StatePack::LIN_VEL_CurToRefInRef() const {
        return SO3_CurToRef * LIN_VEL_CurToRefInCur;
    }

    StateManager::StatePack::StatePack() = default;
}
