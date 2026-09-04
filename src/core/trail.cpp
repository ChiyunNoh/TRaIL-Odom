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

#include "core/trail.h"
#include "cereal/types/utility.hpp"
#include <iomanip>
#include <sstream>

using std::placeholders::_1;

namespace {
    std::string string_from_double(double value, int precision = 3) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }
}

namespace trail {

    TRaIL::TRaIL(const Configor::Ptr &configor)
            : handler(rclcpp::Node::make_shared("trail")), configor(configor),
              dataMagr(DataManager::Create(handler, configor)),
              local_map_(std::make_shared<ns_river::VoxelHashMap>(configor->voxelMap.VoxelSize, configor->voxelMap.MaxRange, configor->voxelMap.MaxPointsPerVoxel)),
              stateMagr(StateManager::Create(dataMagr, configor, local_map_)),
              stateMagrThread(std::make_shared<std::thread>(&StateManager::Run, stateMagr)) {
        /**
         * 1. Once the 'dataMagr' is created, two ros::Subscriber, i.e., 'imuSuber' and 'radarSuber' would
         *    start waiting receive ros message from imu and radar topics;
         * 2. The 'StateManager::Run' runs on another thread once 'stateMagrThread' is created;
         */

        posePublisher     = handler->create_publisher<nav_msgs::msg::Path>(
                                "/path",
                                rclcpp::QoS(10));

        odomPublisher     = handler->create_publisher<nav_msgs::msg::Odometry>(
                                "/axis",
                                rclcpp::QoS(10));

        rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
        map_publisher_ = handler->create_publisher<sensor_msgs::msg::PointCloud2>("/local_map", qos);
        

        const std::string out_dir = configor->dataStream.OutputPath + "/trail_output";
        if (!std::filesystem::exists(out_dir)) {
            if (!std::filesystem::create_directories(out_dir)) {
                throw Status(Status::Flag::WARNING, fmt::format(
                        "the output path for data, i.e., '{}', dose not exist and create failed!", out_dir)
                );
            }
        } else {
            std::filesystem::remove_all(out_dir);
            std::filesystem::create_directories(out_dir);
        }
    }

    TRaIL::Ptr TRaIL::Create(const Configor::Ptr &configor) {
        return std::make_shared<TRaIL>(configor);
    }

    void TRaIL::Run()
    {
        spdlog::info("'TRaIL::Run' has been booted, thread id: {}.", TRAIL_TO_STR(std::this_thread::get_id()));
        auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        exec->add_node(handler);
        std::thread spin_thr([&](){ exec->spin(); });

        rclcpp::Rate rate(configor->preference.IncrementalOptRate);

        while (rclcpp::ok()) {
            auto status = TRaILStatus::GetStatusPackSafely();
            if (TRaILStatus::IsWith(TRaILStatus::StateManager::Status::ShouldQuit, status.StateMagr)) {
                spdlog::warn("'TRaIL::Run' quits normally.");
                break;
            }

            PublishTrailState(status);
            PublishClouds(status);

            if (TRaILStatus::IsWith(TRaILStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                dataMagr->EraseOldDataPieceSafely(status.ValidStateEndTime - 0.2);
            }
            rate.sleep();
        }
        exec->cancel();
        spin_thr.join();
        stateMagrThread->join();
    }

    void TRaIL::save_pose_tum(const std::string& filename, 
                   const std::vector<std::pair<double, Eigen::Vector3d>>& posInW,
                   const std::vector<std::pair<double, Sophus::SO3d>>& quatVec) 
    {
        Eigen::Vector3d position = posInW[0].second;
        assert(posInW.size() == quatVec.size());
        std::ofstream file(filename);
        file << std::fixed << std::setprecision(9);
        double t_prev = posInW[0].first;
        file << t_prev << " " << position.x() << " " << position.y() << " " << position.z() << " ";
        auto q0 = quatVec[0].second.unit_quaternion();
        file << q0.x() << " " << q0.y() << " " << q0.z() << " " << q0.w() << "\n";
        for (size_t i = 1; i < posInW.size(); ++i) {
            double t = posInW[i].first;
        
            const Sophus::SO3d& so3 = quatVec[i].second; //R_wb
            Eigen::Matrix3d R_wb = so3.matrix();  // body to First IMU(R_I0w * R_wb)

            // auto q = so3.unit_quaternion();
            Sophus::SO3d so3_wb(R_wb);
            Eigen::Quaterniond q_wb = so3_wb.unit_quaternion();
            file << t << " "
                << posInW[i].second.x() << " " << posInW[i].second.y() << " " << posInW[i].second.z() << " "
                << q_wb.x() << " " << q_wb.y() << " " << q_wb.z() << " " << q_wb.w() << "\n";
        }
        file.close();

        if(configor->voxelMap.save_map)
        {
            spdlog::info("Saving local map...");
            const auto map = local_map_->Pointcloud();
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.reserve(map.size());
            for (const auto& p : map) {
                cloud.emplace_back(static_cast<float>(p.x()),
                                static_cast<float>(p.y()),
                                static_cast<float>(p.z()));
            }
            cloud.width  = static_cast<uint32_t>(cloud.size());
            cloud.height = 1;
            cloud.is_dense = false;

            const std::string out_dir = configor->dataStream.OutputPath + "/trail_output";

            std::ostringstream oss;
            oss.setf(std::ios::fixed);
            oss.precision(6);
            oss << out_dir << "/local_map" << ".pcd";
            const std::string pcd_path = oss.str();

            int rc = pcl::io::savePCDFileBinary(pcd_path, cloud);
            if (rc == 0) {
                spdlog::info("Saved local map ({} pts) -> {}", cloud.size(), pcd_path);
            } else {
                spdlog::warn("Failed to save local map -> {}", pcd_path);
            }
        }
    }


    void TRaIL::Save() {
        const std::string &tarDir = configor->dataStream.OutputPath + "/trail_output";
        // if (!std::filesystem::exists(tarDir)) {
        //     if (!std::filesystem::create_directories(tarDir)) {
        //         throw Status(Status::Flag::WARNING, fmt::format(
        //                 "the output path for data, i.e., '{}', dose not exist and create failed!", tarDir)
        //         );
        //     }
        // } else {
        //     std::filesystem::remove_all(tarDir);
        //     std::filesystem::create_directories(tarDir);
        // }

        // splines
        auto &splines = this->stateMagr->GetSplines();
        auto &poseSpline = splines->GetRdSpline(Configor::Preference::PoseSpline);
        auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
        const double epoch = configor->preference.OutputResultsWithTimeAligned ? 0.0 : *dataMagr->GetTRaILTimeEpoch();
        const double poseST = poseSpline.MinTime(), poseET = poseSpline.MaxTime();
        const double so3ST = so3Spline.MinTime(), so3ET = so3Spline.MaxTime();

        poseSpline.SetStartTime(poseST + epoch);
        so3Spline.SetStartTime(so3ST + epoch);

        splines->Save(tarDir + "/splines.json");

        poseSpline.SetStartTime(poseST);
        so3Spline.SetStartTime(so3ST);

        // velocity
        const double st = std::max(poseST, so3ST), et = std::min(poseET, so3ET);
        
        std::vector<std::pair<double, Eigen::Vector3d>> posInW, velocityInB;
        std::vector<std::pair<double, Sophus::SO3d>> quatBtoW;
        for (double t = st; t < et;) {
            Eigen::Vector3d position = poseSpline.Evaluate(t);
            posInW.emplace_back(t + epoch, position);

            auto SO3_CurToW = so3Spline.Evaluate(t);
            quatBtoW.emplace_back(t + epoch, SO3_CurToW);

            t += 0.01;
        }
        save_pose_tum(tarDir +"/poses.txt", posInW, quatBtoW);

        {
            // std::ofstream file(tarDir + "/velocity.json", std::ios::out);
            // cereal::JSONOutputArchive ar(file);
            // ar(
            //         cereal::make_nvp("velocity_in_world", velocityInW),
            //         cereal::make_nvp("velocity_in_body", velocityInB)
            // );
        }

        // rotation
        {
            std::ofstream file(tarDir + "/rotation.json", std::ios::out);
            cereal::JSONOutputArchive ar(file);
            ar(cereal::make_nvp("quat_body_to_world", quatBtoW));
        }

        // acce and gyro bias
        {
            std::ofstream file(tarDir + "/bias.json", std::ios::out);
            cereal::JSONOutputArchive ar(file);

            auto acceRecords = stateMagr->GetBaFilter()->GetStateRecords();
            for (auto &item: acceRecords) { item.time += epoch; }

            auto gyroRecords = stateMagr->GetBgFilter()->GetStateRecords();
            for (auto &item: gyroRecords) { item.time += epoch; }

            ar(cereal::make_nvp("acce_bias", acceRecords), cereal::make_nvp("gyro_bias", gyroRecords));
        }

        spdlog::info("outputs of 'TRaIL-Odom' have been saved to dir '{}'.", tarDir);
    }

    
    void TRaIL::log_fancy(double current_time_s, geometry_msgs::msg::PoseStamped& pose_stamped, std::optional<StateManager::StatePack> &status) {

        std::cout<<"\033[2J\033[1;1H"; //clear screen
        std::cout<<"\033[0m" << rpm <<std::endl; 
        std::cout<<"\033[0m"; 

        std::time_t current_time = std::time(nullptr);
        double elapsed_time = current_time_s;

        std::string asc_time = std::asctime(std::localtime(&current_time)); asc_time.pop_back();
        std::cout << "| " << std::left << asc_time;
        std::cout << std::right << std::setfill(' ') << std::setw(35)
        << "Elapsed Time: " + string_from_double(elapsed_time) + " seconds "
        << "|" << std::endl;

        std::cout << "|------------------------------------------------------------|" << std::endl;

        const auto& p = pose_stamped.pose.position;
        std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
                << "Position (x,y,z)    [m] : " 
                + string_from_double(p.x) + " " 
                + string_from_double(p.y) + " " 
                + string_from_double(p.z)
                << "|" << std::endl;

        const auto& o = pose_stamped.pose.orientation;
        const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);

        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); 
        euler *= 180.0 / M_PI;

        std::cout << "| " << std::left << std::setfill(' ') << std::setw(60)
                << "Orientation (r,p,y) [°] : "
                + string_from_double(euler(0)) + " "
                + string_from_double(euler(1)) + " "
                + string_from_double(euler(2))
                << "|" << std::endl;

        std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        << "Lin Velocity {B}  [xyz] : " + string_from_double(status->LIN_VEL_CurToRefInCur(0)) + " "
                                    + string_from_double(status->LIN_VEL_CurToRefInCur(1)) + " "
                                    + string_from_double(status->LIN_VEL_CurToRefInCur(2)) << "|" << std::endl;

        std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        << "Trajectory Length   [m] : " + string_from_double(trajectory_length) << "|" << std::endl;

        std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        << "LiDAR Degeneracy        : kappa=" + string_from_double(status->lidar_kappa)
           + " dim=" + std::to_string(status->degenerate_dim) << "|" << std::endl;

        std::cout << "|------------------------------------------------------------|" << std::endl;


        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        // << "Acc. Bias (x,y,z) [m/s2]  : " + string_from_double(state_point.ba(0)) + " " 
        // + string_from_double(state_point.ba(1)) + " " + string_from_double(state_point.ba(2)) << "|" << std::endl;

        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
        // << "Gyro Bias (x,y,z) [rad/s] : " + string_from_double(state_point.bg(0)) + " " 
        // + string_from_double(state_point.bg(1)) + " " + string_from_double(state_point.bg(2)) << "|" << std::endl;
    
        // std::cout << "|------------------------------------------------------------|" << std::endl;

        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
        // << "Effective Points    [#] : " <<  std::left << std::setw(33) << n_effective_points << "|" << std::endl;

        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
        // << "Intensity Features  [#] : " << std::left << std::setw(6) << n_features << std::left << std::setw(13) 
        // << " Added: " + std::to_string(n_added) << std::left << std::setw(14)<< " Removed: " + std::to_string(n_removed) 
        // << "|" << std::endl;

        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
        // << "Uninformative Dir.  [#] : " << std::left << std::setw(33) << n_uninformative << "|" << std::endl;

        // std::cout << "|------------------------------------------------------------|" << std::endl;

        // double mean_s = timing::Timing::GetMeanSeconds("all");
        // double min_s = timing::Timing::GetMinSeconds("all");
        // double max_s = timing::Timing::GetMaxSeconds("all");
        // std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
        // << "Computation Time    [s] : " << std::left << "Avg: " << string_from_double(mean_s) << std::left 
        // << " Max: " << string_from_double(max_s)  << std::left << " Min: " << string_from_double(min_s) 
        // << " |" << std::endl;
    }

    void TRaIL::PublishTrailState(const TRaILStatus::StatusPack &status) {
        if (!TRaILStatus::IsWith(TRaILStatus::StateManager::Status::NewStateNeedToPublish, status.StateMagr)) {
            return;
        }

        std::optional<StateManager::StatePack> state = this->stateMagr->GetStatePackSafely(
                status.ValidStateEndTime - 0.3
        );
        if (state == std::nullopt) { return; }

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp    = rclcpp::Time(static_cast<int64_t>(state->timestamp * 1e9));
        pose_stamped.header.frame_id = "world";

        pose_stamped.pose.position.x = state->POS_CurToRef(0);
        pose_stamped.pose.position.y = state->POS_CurToRef(1);
        pose_stamped.pose.position.z = state->POS_CurToRef(2);
        pose_stamped.pose.orientation.x = state->SO3_CurToRef.unit_quaternion().x();
        pose_stamped.pose.orientation.y = state->SO3_CurToRef.unit_quaternion().y();
        pose_stamped.pose.orientation.z = state->SO3_CurToRef.unit_quaternion().z();
        pose_stamped.pose.orientation.w = state->SO3_CurToRef.unit_quaternion().w();


        // Accumulate distance from consecutive optimized positions.
        if (!this->pose_path.poses.empty()) {
            const auto &prev_position = this->pose_path.poses.back().pose.position;
            const Eigen::Vector3d segment(
                    pose_stamped.pose.position.x - prev_position.x,
                    pose_stamped.pose.position.y - prev_position.y,
                    pose_stamped.pose.position.z - prev_position.z
            );
            this->trajectory_length += segment.norm();
        }
        this->pose_path.poses.push_back(pose_stamped);

        log_fancy(status.ValidStateEndTime - 0.3, pose_stamped, state);

        
        if ((state->timestamp - this->last_timestamp_pose_pub_) > 1.0 / 5)
        {
            this->pose_path.header.stamp = rclcpp::Time(static_cast<int64_t>(state->timestamp * 1e9));
            this->pose_path.header.frame_id = "world";
            posePublisher->publish(this->pose_path);
            this->last_timestamp_pose_pub_ = state->timestamp;
        }

        nav_msgs::msg::Odometry axis_path;
        axis_path.header.stamp = rclcpp::Time(static_cast<int64_t>(state->timestamp * 1e9));
        axis_path.header.frame_id = "world";
        axis_path.pose.pose.position.x = pose_stamped.pose.position.x;
        axis_path.pose.pose.position.y = pose_stamped.pose.position.y;
        axis_path.pose.pose.position.z = pose_stamped.pose.position.z;
        axis_path.pose.pose.orientation.x = state->SO3_CurToRef.unit_quaternion().x();
        axis_path.pose.pose.orientation.y = state->SO3_CurToRef.unit_quaternion().y();
        axis_path.pose.pose.orientation.z = state->SO3_CurToRef.unit_quaternion().z();
        axis_path.pose.pose.orientation.w = state->SO3_CurToRef.unit_quaternion().w();

        odomPublisher->publish(axis_path);
        {
            LOCK_TRAIL_STATUS
            TRaILStatus::StateManager::CurStatus ^= TRaILStatus::StateManager::Status::NewStateNeedToPublish;
        }
    }

    std::unique_ptr<sensor_msgs::msg::PointCloud2> TRaIL::CreatePointCloud2Msg(const size_t n_points, const std_msgs::msg::Header &header, bool timestamp = false) {
        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        cloud_msg->header = header;
        cloud_msg->fields.clear();
        int offset = 0;
        offset = addPointField(*cloud_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
        offset = addPointField(*cloud_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
        offset = addPointField(*cloud_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
        offset += sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
        if (timestamp) {
            // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
            offset = addPointField(*cloud_msg, "time", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
            offset += sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64);
        }

        // Resize the point cloud accordingly
        cloud_msg->point_step = offset;
        cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
        cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
        modifier.resize(n_points);
        return cloud_msg;
    }

    inline void TRaIL::FillPointCloud2XYZ(const std::vector<Eigen::Vector3d> &points, sensor_msgs::msg::PointCloud2 &msg) {
        sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
        for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
            const Eigen::Vector3d &point = points[i];
            *msg_x = point.x();
            *msg_y = point.y();
            *msg_z = point.z();
        }
    }

    inline std::unique_ptr<sensor_msgs::msg::PointCloud2> TRaIL::EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                       const std_msgs::msg::Header &header) {
        auto msg = CreatePointCloud2Msg(points.size(), header);
        FillPointCloud2XYZ(points, *msg);
        return msg;
    }

    void TRaIL::PublishClouds(const TRaILStatus::StatusPack &status) {
        const auto map = local_map_->Pointcloud(); 
        if(map.empty()) {
            return;
        }
        std::optional<StateManager::StatePack> state = this->stateMagr->GetStatePackSafely(
                status.ValidStateEndTime - 0.1
        );
        if (state == std::nullopt) { return; }
        std_msgs::msg::Header header;
        header.frame_id = "world";
        header.stamp = rclcpp::Time(static_cast<int64_t>(state->timestamp * 1e9));
        map_publisher_->publish(std::move(EigenToPointCloud2(map, header)));
    }
}
