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

#include "sensor/radar_data_loader.h"
#include "util/enum_cast.hpp"
#include "core/status.h"
#include "pcl_conversions/pcl_conversions.h"

namespace trail {

    RadarDataUnpacker::Ptr RadarDataUnpacker::Create() {
        return std::make_shared<RadarDataUnpacker>();
    }

    RadarTargetArray::Ptr RadarDataUnpacker::Unpack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        // for (const auto &item: msg->fields) { std::cout << item.name << ' '; }

        XRIORadarPOSVCloud radarTargets;
        pcl::fromROSMsg(*msg, radarTargets);

        std::vector<RadarTarget::Ptr> targets;
        targets.reserve(radarTargets.size());

        for (const auto &tar: radarTargets) {
            if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) ||
                std::isnan(tar.v_doppler_mps)) { continue; }

            targets.push_back(
                    RadarTarget::Create(rclcpp::Time(msg->header.stamp).seconds(), {tar.x, tar.y, tar.z}, tar.v_doppler_mps)
            );
        }

        return RadarTargetArray::Create(rclcpp::Time(msg->header.stamp).seconds(), targets);
    }

    RadarTargetArray::Ptr RadarDataUnpacker::Unpack_colo(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        // for (const auto &item: msg->fields) { std::cout << item.name << ' '; }

        COLORadarPOSVCloud radarTargets;
        pcl::fromROSMsg(*msg, radarTargets);

        std::vector<RadarTarget::Ptr> targets;
        targets.reserve(radarTargets.size());

        for (const auto &tar: radarTargets) {
            if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) ||
                std::isnan(tar.doppler)) { continue; }

            targets.push_back(
                    RadarTarget::Create(rclcpp::Time(msg->header.stamp).seconds(), {tar.x, tar.y, tar.z}, tar.doppler)
            );
        }

        return RadarTargetArray::Create(rclcpp::Time(msg->header.stamp).seconds(), targets);
    }

    RadarTargetArray::Ptr RadarDataUnpacker::Unpack_coral(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        // for (const auto &item: msg->fields) { std::cout << item.name << ' '; }

        CoralPOSVCloud radarTargets;
        pcl::fromROSMsg(*msg, radarTargets);

        std::vector<RadarTarget::Ptr> targets;
        targets.reserve(radarTargets.size());

        for (const auto &tar: radarTargets) {
            if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) ||
                std::isnan(tar.velocity)) { continue; }

            targets.push_back(
                    RadarTarget::Create(rclcpp::Time(msg->header.stamp).seconds(), {tar.x, tar.y, tar.z}, tar.velocity)
            );
        }
        return RadarTargetArray::Create(rclcpp::Time(msg->header.stamp).seconds(), targets);
    }

    RadarTargetArray::Ptr RadarDataUnpacker::Unpack(const sensor_msgs::msg::PointCloud::ConstSharedPtr &msg) {
        // for (const auto &item: msg->fields) { std::cout << item.name << ' '; }

        // NTURadarCloud radarTargets;
        // pcl::fromROSMsg(*msg, radarTargets);

        std::vector<RadarTarget::Ptr> targets;
        targets.reserve(msg->points.size());
        for (size_t i = 0; i < msg->points.size(); i+=10)
        {
            if (msg->channels[2].values[i] > 0)
            {
                if (msg->points[i].x == NAN || msg->points[i].y == NAN || msg->points[i].z == NAN)
                    continue;
                if (msg->points[i].x == INFINITY || msg->points[i].y == INFINITY || msg->points[i].z == INFINITY)
                    continue;
                targets.push_back(
                    RadarTarget::Create(rclcpp::Time(msg->header.stamp).seconds(), {msg->points[i].x,msg->points[i].y,msg->points[i].z}, msg->channels[0].values[i])
                );
            }
        }
        
        return RadarTargetArray::Create(rclcpp::Time(msg->header.stamp).seconds(), targets);
    }
}
