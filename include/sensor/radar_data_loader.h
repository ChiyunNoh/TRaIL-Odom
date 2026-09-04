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

#ifndef TRAIL_RADAR_DATA_LOADER_H
#define TRAIL_RADAR_DATA_LOADER_H

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor/radar.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

struct RadarPointCloudType
{
  PCL_ADD_POINT4D      // x,y,z position in [m]
  PCL_ADD_INTENSITY;
  union
    {
      struct
      {
        float doppler;
      };
      float data_c[4];
    };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
} EIGEN_ALIGN16; 
POINT_CLOUD_REGISTER_POINT_STRUCT
(
    RadarPointCloudType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, doppler, doppler)
)
using NTURadarCloud = pcl::PointCloud<RadarPointCloudType>;

struct EIGEN_ALIGN16 XRIORadarTarget {
    PCL_ADD_POINT4D;   // quad-word XYZ
    float snr_db;         // CFAR cell to side noise ratio in [dB]
    float noise_db;       // CFAR noise level of the side of the detected cell in [dB]
    float v_doppler_mps;  // Doppler's velocity in [m/s]

    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
        XRIORadarTarget,
        (float, x, x)(float, y, y)(float, z, z)(float, snr_db, snr_db)
                (float, noise_db, noise_db)(float, v_doppler_mps, v_doppler_mps)
)
using XRIORadarPOSVCloud = pcl::PointCloud<XRIORadarTarget>;

struct EIGEN_ALIGN16 COLORadarTarget {
    PCL_ADD_POINT4D;   // quad-word XYZ
    float intensity;         // CFAR cell to side noise ratio in [dB]
    float range;       // CFAR noise level of the side of the detected cell in [dB]
    float doppler;  // Doppler's velocity in [m/s]

    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
        COLORadarTarget,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                (float, range, range)(float, doppler, doppler)
)
using COLORadarPOSVCloud = pcl::PointCloud<COLORadarTarget>;

struct EIGEN_ALIGN16 CoralTarget {
    PCL_ADD_POINT4D;   // quad-word XYZ
    float intensity;         // CFAR cell to side noise ratio in [dB]
    // double az;       // CFAR noise level of the side of the detected cell in [dB]
    float velocity;  // Doppler's velocity in [m/s]

    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
        CoralTarget,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity)
)
using CoralPOSVCloud = pcl::PointCloud<CoralTarget>;

namespace trail {
    enum class RadarMsgType {
        POINTCLOUD2_XRIO,
        NTURADAR,
        COLORADAR,
        CORAL
    };

    class RadarDataUnpacker {
    public:
        using Ptr = std::shared_ptr<RadarDataUnpacker>;

    public:
        explicit RadarDataUnpacker() = default;

        static RadarDataUnpacker::Ptr Create();

        static RadarTargetArray::Ptr Unpack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        static RadarTargetArray::Ptr Unpack_colo(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        static RadarTargetArray::Ptr Unpack_coral(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        static RadarTargetArray::Ptr Unpack(const sensor_msgs::msg::PointCloud::ConstSharedPtr &msg);
    };
}

#endif //TRAIL_RADAR_DATA_LOADER_H
