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

#ifndef TRAIL_CONFIGOR_H
#define TRAIL_CONFIGOR_H

#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/set.hpp"
#include "util/utils.hpp"
#include "util/enum_cast.hpp"
#include "core/calib_param_manager.h"
#include "sensor/imu_data_loader.h"
#include "sensor/radar_data_loader.h"

namespace trail {
    struct Configor {
    public:
        using Ptr = std::shared_ptr<Configor>;

    public:
        struct DataStream {
            std::string IMUTopic;
            std::string IMUMsgType;

            std::string RadarTopic;
            std::string RadarMsgType;

            std::string LidarTopic;
            std::string LidarMsgType;

            CalibParamManager CalibParam;

            std::string OutputPath;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(IMUTopic), CEREAL_NVP(IMUMsgType),
                   CEREAL_NVP(RadarTopic), CEREAL_NVP(LidarTopic), CEREAL_NVP(RadarMsgType), CEREAL_NVP(LidarMsgType),
                   CEREAL_NVP(CalibParam), CEREAL_NVP(OutputPath));
            }
        } dataStream;

        struct Preprocess {
            int n_scans;             
            int timestamp_unit;      
            int point_filter_num;    
            float blind;             
            double imu_acc_scale = 1.0;
        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(n_scans), CEREAL_NVP(timestamp_unit),
                   CEREAL_NVP(point_filter_num), CEREAL_NVP(blind),
                   CEREAL_NVP(imu_acc_scale));
            }
        } preprocess;

        struct VoxelMap {
            double VoxelSize;
            double MaxRange;
            double MinRange;
            unsigned int  MaxPointsPerVoxel;
            bool save_map;
        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(VoxelSize), CEREAL_NVP(MaxRange), CEREAL_NVP(MinRange), CEREAL_NVP(MaxPointsPerVoxel), CEREAL_NVP(save_map));
            }
        } voxelMap;

        struct Prior {
            static constexpr int SplineOrder = 4;
            double GravityDirection;
            double GravityNorm;

            double SO3SplineKnotDist;
            double PoseSplineKnotDist;

            double AcceWeight;
            double AcceBiasRandomWalk;
            double GyroWeight;
            double GyroBiasRandomWalk;
            double RadarWeight;
            double LiDARWeight;

            double CauchyLossForRadarFactor;

            double RotGravWeight;
            double GravityWeight;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(GravityDirection),
                        CEREAL_NVP(GravityNorm),
                        CEREAL_NVP(SO3SplineKnotDist), CEREAL_NVP(PoseSplineKnotDist),
                        CEREAL_NVP(AcceWeight), CEREAL_NVP(AcceBiasRandomWalk),
                        CEREAL_NVP(GyroWeight), CEREAL_NVP(GyroBiasRandomWalk),
                        CEREAL_NVP(RadarWeight), CEREAL_NVP(LiDARWeight),
                        CEREAL_NVP(CauchyLossForRadarFactor),
                        CEREAL_NVP(RotGravWeight), CEREAL_NVP(GravityWeight)
                );
            }
        } prior{};

        struct Preference {
            /**
             * when the mode is 'DEBUG_MODE', then:
             * 1. the solving information from ceres would be output on the console
             * 2.
             */
            static bool DEBUG_MODE;

            static std::string SO3Spline;
            static std::string PoseSpline;

            std::uint32_t IMUMsgQueueSize;
            std::uint32_t RadarMsgQueueSize;
            std::uint32_t LidarMsgQueueSize;
            std::uint32_t IncrementalOptRate;

            bool OutputResultsWithTimeAligned;

            bool VisualizeVelocityInBody;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(IMUMsgQueueSize), CEREAL_NVP(RadarMsgQueueSize), CEREAL_NVP(LidarMsgQueueSize),
                        CEREAL_NVP(IncrementalOptRate),
                        CEREAL_NVP(OutputResultsWithTimeAligned), CEREAL_NVP(VisualizeVelocityInBody)
                );
            }
        } preference{};

    public:
        Configor();

        static Ptr Create();

        // load configure information from the xml file
        static Configor::Ptr
        Load(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

        // load configure information from the xml file
        void Save(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

        // print the main fields
        void PrintMainFields();

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(
                    cereal::make_nvp("DataStream", dataStream),
                    cereal::make_nvp("Preprocess", preprocess),
                    cereal::make_nvp("Prior", prior),
                    cereal::make_nvp("VoxelMap", voxelMap),
                    cereal::make_nvp("Preference", preference)
            );
        }
    };
}


#endif //TRAIL_CONFIGOR_H
