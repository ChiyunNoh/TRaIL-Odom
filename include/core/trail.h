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

#ifndef TRAIL_TRAIL_H
#define TRAIL_TRAIL_H
#include "config/configor.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "core/state_manager.h"
#include "voxelmap/VoxelHashMap.hpp"
#include <fstream>
#include <nav_msgs/msg/path.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace trail {
    class TRaIL {
    public:
        using Ptr = std::shared_ptr<TRaIL>;
    private:
        // ros-related members
        rclcpp::Node::SharedPtr handler;
        Configor::Ptr configor;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             posePublisher;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odomPublisher;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;

        DataManager::Ptr dataMagr;
        std::shared_ptr<ns_river::VoxelHashMap> local_map_;
        StateManager::Ptr stateMagr;
        

        std::shared_ptr<std::thread> stateMagrThread;

        double last_timestamp_pose_pub_ = 0.0;
        double trajectory_length = 0.0;

        nav_msgs::msg::Path pose_path;

        std::string rpm = R"(                               
  __________        ______       ____      __              
 /_  __/ __ \____ _/  _/ /      / __ \____/ /___  ____ ___ 
  / / / /_/ / __ `// // /      / / / / __  / __ \/ __ `__ \
 / / / _, _/ /_/ // // /___   / /_/ / /_/ / /_/ / / / / / /
/_/ /_/ |_|\__,_/___/_____/   \____/\__,_/\____/_/ /_/ /_/                                                           
        )"; 

    public:
        explicit TRaIL(const Configor::Ptr &configor);

        static Ptr Create(const Configor::Ptr &configor);

        void Run();

        void save_pose_tum(const std::string& filename, 
                   const std::vector<std::pair<double, Eigen::Vector3d>>& velocity,
                   const std::vector<std::pair<double, Sophus::SO3d>>& quatVec);
        
        void log_fancy(double current_time_s, geometry_msgs::msg::PoseStamped& pose_stamped, std::optional<StateManager::StatePack> &status);

        void Save();

    protected:
        void PublishTrailState(const TRaILStatus::StatusPack &status);
        std::unique_ptr<sensor_msgs::msg::PointCloud2> CreatePointCloud2Msg(const size_t n_points, const std_msgs::msg::Header &header,bool timestamp);
        void FillPointCloud2XYZ(const std::vector<Eigen::Vector3d> &points, sensor_msgs::msg::PointCloud2 &msg);
        std::unique_ptr<sensor_msgs::msg::PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                        const std_msgs::msg::Header &header);
        void PublishClouds(const TRaILStatus::StatusPack &status);
    };
}

#endif //TRAIL_TRAIL_H
