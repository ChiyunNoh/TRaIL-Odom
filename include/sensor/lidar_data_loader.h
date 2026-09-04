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

#ifndef TRAIL_LIDAR_DATA_LOADER_H
#define TRAIL_LIDAR_DATA_LOADER_H

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor/lidar.h"
#include "config/configor.h"
#include "util/utils.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h> 

namespace ouster_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;
}  

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
)

namespace hesai_ros {

    struct EIGEN_ALIGN16 Point
    {
    PCL_ADD_POINT4D;          // x, y, z, padding (w)
    float intensity;          // field: intensity (FLOAT32)
    std::uint16_t ring;       // field: ring (UINT16)
    std::uint16_t _pad;       // alignment padding (optional but recommended)
    double timestamp;         // field: timestamp (FLOAT64)

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    } // namespace hesai_ros

    POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace mid_ros {

    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;              // x, y, z, padding
        float intensity;              // datatype: 7 (FLOAT32)
        std::uint8_t tag;             // datatype: 2 (UINT8)
        std::uint8_t line;            // datatype: 2 (UINT8)
        double timestamp;             // datatype: 8 (FLOAT64)

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

} // namespace mid_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(mid_ros::Point,
    (float,        x,         x)
    (float,        y,         y)
    (float,        z,         z)
    (float,        intensity, intensity)
    (std::uint8_t, tag,       tag)
    (std::uint8_t, line,      line)
    (double,       timestamp, timestamp)
)

enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};

namespace trail {
    enum class LidarMsgType {
        LIDAR_OUSTER,
        LIDAR_MID360,
        LIDAR_HESAI,
        LIDAR_RAI
    };

    class LidarDataUnpacker {
    public:
        using Ptr = std::shared_ptr<LidarDataUnpacker>;

    public:
        explicit LidarDataUnpacker() = default;

        static LidarDataUnpacker::Ptr Create();

        static LidarTarget::Ptr Unpack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, 
                                        int timestamp_unit, int point_filter_num, float blind);

        static LidarTarget::Ptr Unpack_MID(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, 
                                        int timestamp_unit, int point_filter_num, float blind);
        
        static LidarTarget::Ptr Unpack_RAI(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, 
                                        int timestamp_unit, int point_filter_num, float blind);                                

        static LidarTarget::Ptr Unpack_HESAI(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, 
                                        int timestamp_unit, int point_filter_num, float blind);
        
    };
}

#endif 
