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

#include "config/configor.h"

namespace trail {
    // -------------------
    // static member field
    // -------------------
    std::string Configor::Preference::SO3Spline = "so3";
    std::string Configor::Preference::PoseSpline = "pos";

    bool Configor::Preference::DEBUG_MODE = false;

    Configor::Configor() = default;

    Configor::Ptr Configor::Create() {
        return std::make_shared<Configor>();
    }

    Configor::Ptr Configor::Load(const std::string &filename, CerealArchiveType::Enum archiveType) {
        auto configor = Configor::Create();
        std::ifstream file(filename, std::ios::in);
        auto ar = GetInputArchiveVariant(file, archiveType);
        SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("Configor", *configor));
        return configor;
    }

    void Configor::Save(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ofstream file(filename, std::ios::out);
        auto ar = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("Configor", *this));
    }

    void Configor::PrintMainFields() {
        constexpr const char* DESC_FORMAT = "\n{:>35}: {}";

        fmt::memory_buffer buf;
        fmt::format_to(buf, "main fields of Configor");
        fmt::format_to(buf, "\n{}", std::string(60, '-'));

        auto section = [&](std::string_view title) {
                fmt::format_to(buf, "\n\n[{}]", title);
        };
        auto put = [&](std::string_view name, const auto& value) {
                fmt::format_to(buf, DESC_FORMAT, name, value);
        };
        #define P(field) put(#field, field)

        // --- DataStream ---
        section("DataStream");
        P(dataStream.IMUTopic);
        P(dataStream.IMUMsgType);
        P(dataStream.RadarTopic);
        P(dataStream.RadarMsgType);
        P(dataStream.LidarTopic);
        P(dataStream.LidarMsgType);
        P(dataStream.OutputPath);

        // --- Preprocess ---
        section("Preprocess");
        P(preprocess.n_scans);
        P(preprocess.timestamp_unit);
        P(preprocess.point_filter_num);
        P(preprocess.blind);
        P(preprocess.imu_acc_scale);

        // --- VoxelMap ---
        section("VoxelMap");
        P(voxelMap.VoxelSize);
        P(voxelMap.MaxRange);
        P(voxelMap.MinRange);
        P(voxelMap.MaxPointsPerVoxel);
        P(voxelMap.save_map);

        // --- Prior ---
        section("Prior");
        P(prior.SplineOrder);
        P(prior.GravityDirection);
        P(prior.GravityNorm);
        P(prior.SO3SplineKnotDist);
        P(prior.PoseSplineKnotDist);
        P(prior.AcceWeight);
        P(prior.AcceBiasRandomWalk);
        P(prior.GyroWeight);
        P(prior.GyroBiasRandomWalk);
        P(prior.RadarWeight);
        P(prior.CauchyLossForRadarFactor);

        // --- Preference ---
        section("Preference");
        P(preference.IMUMsgQueueSize);
        P(preference.RadarMsgQueueSize);
        P(preference.LidarMsgQueueSize);
        P(preference.IncrementalOptRate);
        P(preference.OutputResultsWithTimeAligned);

        #undef P

        spdlog::info("{}", fmt::to_string(buf));

        dataStream.CalibParam.ShowParamStatus();
        }

}// namespace trail
