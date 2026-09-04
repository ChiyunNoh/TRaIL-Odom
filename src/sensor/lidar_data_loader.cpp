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

#include "sensor/lidar_data_loader.h"
#include "util/enum_cast.hpp"
#include "core/status.h"
#include "pcl_conversions/pcl_conversions.h"

namespace trail {
    LidarDataUnpacker::Ptr LidarDataUnpacker::Create() {
        return std::make_shared<LidarDataUnpacker>();
    }

    LidarTarget::Ptr LidarDataUnpacker::Unpack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                                            int timestamp_unit, int point_filter_num, float blind) {
        
        float time_unit_scale;
        switch (timestamp_unit)
        {
            case SEC:
            time_unit_scale = 1.f;
            break;
            case MS:
            time_unit_scale = 1.e-3f;
            break;
            case US:
            time_unit_scale = 1.e-6f;
            break;
            case NS:
            time_unit_scale = 1.e-9f;
            break;
            default:
            time_unit_scale = 1.f;
            break;
        }

        pcl::PointCloud<ouster_ros::Point>::Ptr ousterTargets(new pcl::PointCloud<ouster_ros::Point>());
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        pcl::fromROSMsg(*msg, *ousterTargets);
        size_t plsize = ousterTargets->size();
        PointCloudXYZIN::Ptr cloud(new PointCloudXYZIN());
        cloud->is_dense = false;
        cloud->resize(plsize);
        if (plsize == 0) return nullptr;
        
        std::size_t j = 0;
        for (unsigned int i = 0; i < plsize; ++i) {
            if (i % point_filter_num == 0) {
                PointType pt;
                pt.x = ousterTargets->points[i].x;
                pt.y = ousterTargets->points[i].y;
                pt.z = ousterTargets->points[i].z;
                pt.intensity = ousterTargets->points[i].intensity;
                pt.curvature = ousterTargets->points[i].t * time_unit_scale;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind)) {
                    cloud->at(j++) = pt;
                }
            }
        }
        cloud->resize(j);
        return LidarTarget::Create(std::pair<double,double>{timestamp, timestamp}, cloud);
    }

    LidarTarget::Ptr LidarDataUnpacker::Unpack_MID(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                                            int timestamp_unit, int point_filter_num, float blind) {

        double time_unit_scale;
        switch (timestamp_unit)
        {
            case SEC:
            time_unit_scale = 1.f;
            break;
            case MS:
            time_unit_scale = 1.e-3f;
            break;
            case US:
            time_unit_scale = 1.e-6f;
            break;
            case NS:
            time_unit_scale = 1.e-9d;
            break;
            default:
            time_unit_scale = 1.f;
            break;
        }

        pcl::PointCloud<mid_ros::Point>::Ptr mid360Targets(new pcl::PointCloud<mid_ros::Point>());

        pcl::fromROSMsg(*msg, *mid360Targets);
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        size_t plsize = mid360Targets->size();
        PointCloudXYZIN::Ptr cloud(new PointCloudXYZIN());
        cloud->is_dense = false;
        cloud->resize(plsize);
        if (plsize == 0) return nullptr;

        std::size_t j = 0;
        for (unsigned int i = 0; i < plsize; ++i) {
            if(i==100){
                // spdlog::info("First point timestamp: {:.18f}", mid360Targets->points[i].timestamp);
                // spdlog::info("timetamp unit scale: {}", time_unit_scale);
                // spdlog::info("header timestamp: {}", timestamp);
                // spdlog::info("timestamp: {}", mid360Targets->points[i].timestamp * time_unit_scale - timestamp);   
            }
            if (i % point_filter_num == 0) {
                PointType pt;
                pt.x = mid360Targets->points[i].x;
                pt.y = mid360Targets->points[i].y;
                pt.z = mid360Targets->points[i].z;
                pt.intensity = mid360Targets->points[i].intensity;
                const double pt_sec = static_cast<double>(mid360Targets->points[i].timestamp) * time_unit_scale;
                pt.curvature = pt_sec - timestamp;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind)) {
                    cloud->at(j++) = pt;
                }
            }
        }
        cloud->resize(j);
        return LidarTarget::Create(std::pair<double,double>{timestamp, timestamp}, cloud);
    }

    LidarTarget::Ptr LidarDataUnpacker::Unpack_RAI(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                                            int timestamp_unit, int point_filter_num, float blind) {

        double time_unit_scale;
        switch (timestamp_unit)
        {
            case SEC:
            time_unit_scale = 1.f;
            break;
            case MS:
            time_unit_scale = 1.e-3f;
            break;
            case US:
            time_unit_scale = 1.e-6f;
            break;
            case NS:
            time_unit_scale = 1.e-9d;
            break;
            default:
            time_unit_scale = 1.f;
            break;
        }

        pcl::PointCloud<mid_ros::Point>::Ptr mid360Targets(new pcl::PointCloud<mid_ros::Point>());

        pcl::fromROSMsg(*msg, *mid360Targets);
        double timestamp = mid360Targets->points[0].timestamp;
        size_t plsize = mid360Targets->size();
        PointCloudXYZIN::Ptr cloud(new PointCloudXYZIN());
        cloud->is_dense = false;
        cloud->resize(plsize);
        if (plsize == 0) return nullptr;

        std::size_t j = 0;
        for (unsigned int i = 0; i < plsize; ++i) {
            if(i==100){
                // spdlog::info("First point timestamp: {:.18f}", mid360Targets->points[i].timestamp);
                // spdlog::info("timetamp unit scale: {}", time_unit_scale);
                // spdlog::info("header timestamp: {}", timestamp);
                // spdlog::info("timestamp: {}", mid360Targets->points[i].timestamp * time_unit_scale - timestamp);   
            }
            if (i % point_filter_num == 0) {
                PointType pt;
                pt.x = mid360Targets->points[i].x;
                pt.y = mid360Targets->points[i].y;
                pt.z = mid360Targets->points[i].z;
                pt.intensity = mid360Targets->points[i].intensity;
                const double pt_sec = static_cast<double>(mid360Targets->points[i].timestamp) * time_unit_scale;
                pt.curvature = pt_sec - timestamp;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind)) {
                    cloud->at(j++) = pt;
                }
            }
        }
        cloud->resize(j);
        return LidarTarget::Create(std::pair<double,double>{timestamp, timestamp}, cloud);
    }


    LidarTarget::Ptr LidarDataUnpacker::Unpack_HESAI(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                                            int timestamp_unit, int point_filter_num, float blind) {

        double time_unit_scale;
        switch (timestamp_unit)
        {
            case SEC:
            time_unit_scale = 1.f;
            break;
            case MS:
            time_unit_scale = 1.e-3f;
            break;
            case US:
            time_unit_scale = 1.e-6f;
            break;
            case NS:
            time_unit_scale = 1.e-9d;
            break;
            default:
            time_unit_scale = 1.f;
            break;
        }

        pcl::PointCloud<hesai_ros::Point>::Ptr hesaiTargets(new pcl::PointCloud<hesai_ros::Point>());

        pcl::fromROSMsg(*msg, *hesaiTargets);
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        size_t plsize = hesaiTargets->size();
        PointCloudXYZIN::Ptr cloud(new PointCloudXYZIN());
        cloud->is_dense = false;
        cloud->resize(plsize);
        if (plsize == 0) return nullptr;

        std::size_t j = 0;
        for (unsigned int i = 0; i < plsize; ++i) {
            if (i % point_filter_num == 0) {
                PointType pt;
                pt.x = hesaiTargets->points[i].x;
                pt.y = hesaiTargets->points[i].y;
                pt.z = hesaiTargets->points[i].z;
                pt.intensity = hesaiTargets->points[i].intensity;
                const double pt_sec = static_cast<double>(hesaiTargets->points[i].timestamp) * time_unit_scale;
                pt.curvature = pt_sec - timestamp;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind)) {
                    cloud->at(j++) = pt;
                }
            }
        }
        cloud->resize(j);
        return LidarTarget::Create(std::pair<double,double>{timestamp, timestamp}, cloud);
    }

}

