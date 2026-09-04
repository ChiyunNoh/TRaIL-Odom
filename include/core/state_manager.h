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

#ifndef TRAIL_STATE_MANAGER_H
#define TRAIL_STATE_MANAGER_H

#include <utility>
#include "core/data_manager.h"
#include "core/estimator.h"
#include "voxelmap/VoxelHashMap.hpp"
#include "rms/rms.h"
#include "ctraj/factor/marginalization_factor.h"
#include "core/status.h"
#include "core/bias_filter.h"
#include <tuple>

namespace trail {
#define LOCK_STATES std::unique_lock<std::mutex> statesLock(StateManager::StatesMutex);

    class StateManager {
    public:
        using Ptr = std::shared_ptr<StateManager>;
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

        using Vector3dVector   = std::vector<Eigen::Vector3d>;
        using Timestamps       = std::vector<double>;

        using PointsTsPair     = std::pair<Vector3dVector, Timestamps>;
        using PointsTsTuple    = std::tuple<PointsTsPair, PointsTsPair>;
        using Matrix6d = Eigen::Matrix<double, 6, 6>;
        using Vector6d = Eigen::Matrix<double, 6, 1>;

        using LinearSystem = std::pair<Matrix6d, Vector6d>;

        

        struct StatePack {
        public:
            double timestamp{};
            Sophus::SO3d SO3_CurToRef;
            Eigen::Vector3d POS_CurToRef;
            Eigen::Vector3d LIN_VEL_CurToRefInCur;
            Eigen::Vector3d gravity;
            Eigen::Vector3d ba;
            Eigen::Vector3d bg;
            double lidar_kappa{std::numeric_limits<double>::infinity()};
            int degenerate_dim{0};

            StatePack(double timestamp, const Sophus::SO3d &so3CurToRef, Eigen::Vector3d pose_CurToRef,Eigen::Vector3d linVelCurToRefInCur,
                      Eigen::Vector3d gravity, Eigen::Vector3d ba, Eigen::Vector3d bg);

            StatePack();

            [[nodiscard]] Eigen::Vector3d LIN_VEL_CurToRefInRef() const;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(timestamp), CEREAL_NVP(SO3_CurToRef), CEREAL_NVP(POS_CurToRef),
                        CEREAL_NVP(LIN_VEL_CurToRefInCur), CEREAL_NVP(gravity),
                        CEREAL_NVP(ba), CEREAL_NVP(bg)
                );
            }
        };

        std::vector<Sophus::SO3d> Rv{Sophus::SO3d{}}; // relative rotation between two consecutive ***body frames*** B_{k} -> B_{k+1}, 0th element is identity
        std::vector<Eigen::Vector3d> Tv{Eigen::Vector3d::Zero()}; // relative translation between two consecutive ***body frames*** B_{k} -> B_{k+1}, 0th element is zero
        std::vector<double> lidar_scan_times{0.0};

        size_t last_lidar = 0;

    private:
        DataManager::Ptr dataMagr;
        Configor::Ptr configor;
        std::shared_ptr<ns_river::VoxelHashMap> local_map_;

        int success_cnt_ = 0;
        int fail_cnt_ = 0;
        int icp_iter_cnt_ = 0;
        double latest_lidar_kappa_ = std::numeric_limits<double>::infinity();
        int latest_degenerate_dim_ = 0;

        // spline bundle containing rotation and position splines
        SplineBundleType::Ptr splines;
        // the gravity represented in the world frame
        std::shared_ptr<Eigen::Vector3d> gravity;
        // the biases of acceleration and gyroscope
        std::shared_ptr<Eigen::Vector3d> ba, bg;

        Sophus::SE3d last_pose_;
        Sophus::SE3d last_delta_;

        ns_ctraj::MarginalizationInfo::Ptr margInfo;

        BiasFilter::Ptr baFilter;
        BiasFilter::Ptr bgFilter;

        Sophus::SO3d SO3_RefToW;

        std::map<int, double *> lastKeepSo3KnotAdd;
        std::map<int, double *> lastKeepPosKnotAdd;

        size_t ICP_MAX_ITER = 3;

    public:
        // mutexes employed in multi-thread framework
        static std::mutex StatesMutex;

    public:
        StateManager(DataManager::Ptr dataMagr,
                 Configor::Ptr configor,
                 std::shared_ptr<ns_river::VoxelHashMap> voxel_map);

        static Ptr Create(const DataManager::Ptr &dataMagr,
                        const Configor::Ptr &configor,
                        const std::shared_ptr<ns_river::VoxelHashMap> &voxel_map);

        void Run();

        const Sophus::SO3d& GetSO3_RefToW() const { return SO3_RefToW; }

        void buildRadarSensitivityMatrix(const Eigen::MatrixXd &degenerate_basis_radar, std::list<RadarTarget::Ptr> &inlier_radarData, Eigen::MatrixXd &Z);

        bool isRadarSubspaceObservable(const Eigen::MatrixXd &Z);

        void reweightingRadarMeasurement(std::vector<double> &radar_weight_normalize, const Eigen::MatrixXd &Z, double eta);

        PointsTsTuple Voxelize(const std::vector<Eigen::Vector3d> &frame, const std::vector<double> &timestamps) const;

        void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points);

        LinearSystem BuildLinearSystem(const std::vector<Eigen::Vector3d> &points,
                                                        double kernel,
                                                        double alpha);

        Sophus::SE3d AlignPointToPlane(const StateManager::PointsTsPair &frame_pair,
                                                                                                   const Sophus::SE3d &initial_guess,
                                                                                                   double kernel);

        Estimator::PlaneMatches KNNDataAssociation(const PointsTsPair &frame_pair, std::vector<int> &sampled_indices,
                             std::size_t K,
                             double neighbor_sqdist_thresh ,   
                             double plane_point_res_thresh);
        
        Estimator::PlaneMatches KNNDataAssociation(const PointsTsPair &frame_pair,
                             std::size_t K,
                             double neighbor_sqdist_thresh ,   
                             double plane_point_res_thresh);

        Estimator::PlaneMatches KNNDataAssociation(const std::vector<Eigen::Vector3d> &points,
                             std::size_t K,
                             double neighbor_sqdist_thresh ,   
                             double plane_point_res_thresh);


        [[nodiscard]] std::optional<StatePack> GetStatePackSafely(double t) const;

        [[nodiscard]] std::vector<std::optional<StatePack>> GetStatePackSafely(const std::vector<double> &times) const;

        [[nodiscard]] const SplineBundleType::Ptr &GetSplines() const;

        [[nodiscard]] const BiasFilter::Ptr &GetBaFilter() const;

        [[nodiscard]] const BiasFilter::Ptr &GetBgFilter() const;

    protected:
        // --------------
        // initialization
        // --------------

        bool TryPerformInitialization();

        void TryPerformICP();

        [[nodiscard]] inline SplineBundleType::Ptr CreateSplines(double sTime, double eTime) const;

        void InitializeSO3Spline(const std::list<IMUFrame::Ptr> &imuData, const std::vector<Sophus::SE3d> &T_B_rel, const std::vector<double> &lidar_scan_times);

        void InitializePoseSpline(const std::list<IMUFrame::Ptr> &imuData,
                                  const std::vector<Sophus::SE3d> &T_B_rel,
                                  const std::vector<double> &lidar_scan_times);

        void InitializeGravity(const std::list<RadarTargetArray::Ptr> &radarTarAryVec,
                               const std::list<IMUFrame::Ptr> &imuData);

        Estimator::Ptr InitializeBothSpline(const std::list<IMUFrame::Ptr> &imuData,
                                            const std::vector<Sophus::SE3d> &T_B_rel,
                                            const std::vector<double> &lidar_scan_times);

        void TransformLidarPointsToWorldFrame(const PointsTsPair &frame_pair, std::vector<Eigen::Vector3d> &points_in_world);

        void TransformLidarPointsToIMUFrame(const PointsTsPair &frame_pair, std::vector<Eigen::Vector3d> &points_in_body);

        void TransformLidarPointsToIMUFrame(const LidarTarget::Ptr &lidarData, std::vector<Eigen::Vector3d> &points_in_body);

        void AlignInitializedStates();

        void MarginalizationInInit(const Estimator::Ptr &estimator);

        void InitializeBiasFilters(double sTime);

        void UpdateBiasFilters(const Estimator::Ptr &estimator, double eTime);

        // ---------
        // front end
        // ---------
        bool IncrementalOptimization(const TRaILStatus::StatusPack &status);

        void PreOptimization(const std::list<IMUFrame::Ptr> &imuData, double stime, double etime);

        static auto ExtractRange(const std::list<IMUFrame::Ptr> &data, double st, double et) {
            auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > st;
            });
            auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < et;
            }).base();
            return std::pair(sIter, eIter);
        }

        // -----------------------------
        // small help template functions
        // -----------------------------

        template<int Dime1, int Dime2>
        inline std::optional<Eigen::Matrix<double, Dime1, Dime2, Eigen::RowMajor>>
        ObtainVarMatFromEstimator(const std::pair<const double *, const double *> &par, const Estimator::Ptr &est) {
            // compute the covariance of a parameter pair based on the estimator
            ceres::Covariance covariance({});
            auto res = covariance.Compute({par}, est.get());
            if (res) {
                Eigen::Matrix<double, Dime1, Dime2, Eigen::RowMajor> cov;
                covariance.GetCovarianceBlock(par.first, par.second, cov.data());
                return cov;
            } else {
                return {};
            }
        }

        static void LinearExtendKnotTo(SplineBundleType::RdSplineType &spline, double t);

        static void LinearExtendKnotTo(SplineBundleType::So3SplineType &spline, double t);
    };
}


#endif //TRAIL_STATE_MANAGER_H
