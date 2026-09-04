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

#include "core/data_manager.h"

namespace trail {
    // -------------------
    // static member field
    // -------------------
    std::mutex DataManager::IMUDataSeqMutex = {};
    std::mutex DataManager::RadarDataSeqMutex = {};
    std::mutex DataManager::LidarDataSeqMutex = {};
    std::mutex DataManager::RadarInitFramesMutex = {};

    DataManager::DataManager(const rclcpp::Node::SharedPtr &handler, const trail::Configor::Ptr &configor)
            : handler(handler), configor(configor), TRAIL_TIME_EPOCH(std::optional<double>()) {
        spdlog::info("'DataManager' has been booted, thread id: {}.", TRAIL_TO_STR(std::this_thread::get_id()));
        // -------------------------------------------------------
        // create imu message subscriber based on the message type
        // -------------------------------------------------------

        auto cg_imu   = handler->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto cg_radar = handler->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto cg_lidar = handler->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto make_opt = [](rclcpp::CallbackGroup::SharedPtr cg){
            rclcpp::SubscriptionOptions opt;
            opt.callback_group = cg;
            return opt;
        };



        IMUMsgType imuMsgType;
        try {
            imuMsgType = EnumCast::stringToEnum<IMUMsgType>(configor->dataStream.IMUMsgType);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported IMU Type: '{}'. "
                            "Currently supported IMU types are: \n"
                            "(1) SENSOR_IMU: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html\n"
                            "(2)    SBG_IMU: https://github.com/SBG-Systems/sbg_ros_driver.git\n"
                            "...\n"
                            "If you need to use other IMU types, "
                            "please 'Issues' us on the profile of the github repository.",
                            configor->dataStream.IMUMsgType
                    )
            );
        }
        /**
         * create subscriber of imu data
         * @topic configor->dataStream.IMUTopic
         * @queue_size configor->preference.IMUMsgQueueSize
         * @callback HandleIMUMessage(...)
         */
        auto qos_imu = rclcpp::SensorDataQoS().keep_last(50).best_effort();
        switch (imuMsgType) {
            case IMUMsgType::SENSOR_IMU:
                imuSuber = handler->create_subscription<sensor_msgs::msg::Imu>(
                configor->dataStream.IMUTopic,
                qos_imu,
                std::bind(&DataManager::HandleIMUMessage<sensor_msgs::msg::Imu>, this, _1),
                make_opt(cg_imu));
                break;
        }
        // ---------------------------------------------------------
        // create radar message subscriber based on the message type
        // ---------------------------------------------------------
        RadarMsgType radarMsgType;
        try {
            radarMsgType = EnumCast::stringToEnum<RadarMsgType>(configor->dataStream.RadarMsgType);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported Radar Type: '{}'. "
                            "Currently supported radar types are: \n"
                            "(1)    POINTCLOUD2_XRIO: 'sensor_msgs/PointCloud2' with point format: [x, y, z, snr_db, noise_db, v_doppler_mps]\n"
                            "(2)         COLORADAR: 'sensor_msgs/PointCloud2' with point format: [x, y, z, intensity, range, doppler]\n"
                            "(3)             CORAL: 'sensor_msgs/PointCloud2' with point format: [x, y, z, intensity, velocity]\n"
                            "...\n"
                            "If you need to use other radar types, "
                            "please 'Issues' us on the profile of the github repository.",
                            configor->dataStream.RadarMsgType
                    )
            );
        }
        /**
         * create subscriber of radar data
         * @topic configor->dataStream.RadarTopic
         * @queue_size configor->preference.RadarMsgQueueSize
         * @callback HandleRadarMessage(...)
         */
        auto qos_radar = rclcpp::SensorDataQoS().keep_last(50).best_effort();

        switch (radarMsgType) {
            case RadarMsgType::POINTCLOUD2_XRIO:
                radarSuber = handler->create_subscription<sensor_msgs::msg::PointCloud2>(
                configor->dataStream.RadarTopic,
                qos_radar,
                std::bind(&DataManager::HandleRadarMessage<sensor_msgs::msg::PointCloud2>, this, _1),
                make_opt(cg_radar));
                break;
            case RadarMsgType::COLORADAR:
                radarSuber = handler->create_subscription<sensor_msgs::msg::PointCloud2>(
                configor->dataStream.RadarTopic,
                qos_radar,
                std::bind(&DataManager::HandleRadarMessage_colo<sensor_msgs::msg::PointCloud2>, this, _1),
                make_opt(cg_radar));
                break;
            case RadarMsgType::CORAL:
                radarSuber = handler->create_subscription<sensor_msgs::msg::PointCloud2>(
                configor->dataStream.RadarTopic,
                qos_radar,
                std::bind(&DataManager::HandleRadarMessage_coral<sensor_msgs::msg::PointCloud2>, this, _1),
                make_opt(cg_radar));
                break;
        }


        LidarMsgType lidarMsgType;
        try {
            lidarMsgType = EnumCast::stringToEnum<LidarMsgType>(configor->dataStream.LidarMsgType);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported LiDAR Type: '{}'. ",
                            configor->dataStream.LidarMsgType
                    )
            );
        }
        auto qos_lidar = rclcpp::SensorDataQoS().keep_last(50).best_effort();
        switch (lidarMsgType) {
            case LidarMsgType::LIDAR_OUSTER:
                std::cout<<"Lidar subscriber has been created."<<std::endl;
                lidarSuber = handler->create_subscription<sensor_msgs::msg::PointCloud2>(
                configor->dataStream.LidarTopic,
                qos_lidar,
                std::bind(&DataManager::HandleLidarMessage<sensor_msgs::msg::PointCloud2>, this, _1),
                make_opt(cg_lidar));
                break;
            case LidarMsgType::LIDAR_MID360:
                std::cout<<"Lidar subscriber has been created."<<std::endl;
                lidarSuber = handler->create_subscription<sensor_msgs::msg::PointCloud2>(
                configor->dataStream.LidarTopic,
                qos_lidar,
                std::bind(&DataManager::HandleLidarMessage_MID<sensor_msgs::msg::PointCloud2>, this, _1),
                make_opt(cg_lidar));
                break;
            case LidarMsgType::LIDAR_RAI:
                std::cout<<"Lidar subscriber has been created."<<std::endl;
                lidarSuber = handler->create_subscription<sensor_msgs::msg::PointCloud2>(
                configor->dataStream.LidarTopic,
                qos_lidar,
                std::bind(&DataManager::HandleLidarMessage_RAI<sensor_msgs::msg::PointCloud2>, this, _1),
                make_opt(cg_lidar));
                break;
            case LidarMsgType::LIDAR_HESAI:
                std::cout<<"Lidar subscriber has been created."<<std::endl;
                lidarSuber = handler->create_subscription<sensor_msgs::msg::PointCloud2>(
                configor->dataStream.LidarTopic,
                qos_lidar,
                std::bind(&DataManager::HandleLidarMessage_HESAI<sensor_msgs::msg::PointCloud2>, this, _1),
                make_opt(cg_lidar));
                break;
        }


    }

    DataManager::Ptr DataManager::Create(const rclcpp::Node::SharedPtr &handler, const Configor::Ptr &configor) {
        return std::make_shared<DataManager>(handler, configor);
    }

    void DataManager::ShowDataStatus() const {
        std::size_t s = 0;
        double st = 0.0, et = 0.0;
        {
            // lock imu data and obtain its size, start time, and end time quickly
            LOCK_IMU_DATA_SEQ
            if (!imuDataSeq.empty()) {
                s = imuDataSeq.size();
                st = imuDataSeq.front()->GetTimestamp();
                et = imuDataSeq.back()->GetTimestamp();
            }
        }
        spdlog::info("imu data size: {:02}, time span from '{:.6f}' to '{:.6f}'", s, st, et);

        s = 0, st = 0.0, et = 0.0;
        {
            // lock radar data and obtain its size, start time, and end time quickly
            LOCK_RADAR_DATA_SEQ
            if (!radarDataSeq.empty()) {
                s = radarDataSeq.size();
                st = radarDataSeq.front()->GetTimestamp();
                et = radarDataSeq.back()->GetTimestamp();
            }
        }
        spdlog::info("radar data size: {:02}, time span from '{:.6f}' to '{:.6f}'", s, st, et);
    }

    void DataManager::LidarPointTimeCorrection(const LidarTarget::Ptr &lidar_scan){
        auto [start_time, end_time] = lidar_scan->GetTimestamp();
        auto cloud = lidar_scan->GetScan();
        int plsize = cloud->size();
        for (int j = 0; j < plsize; ++j){
            cloud->at(j).curvature += start_time;
            if (cloud->at(j).curvature > end_time){
                end_time = cloud->at(j).curvature;
            }
        }


        auto [t0, t1] = lidar_scan->GetTimestamp();
        lidar_scan->SetTimestamp(
            { t0, end_time}
        );
    }

    void DataManager::OrganizeRadarTarAryForInit(const RadarTargetArray::Ptr &rawTarAry) {
        // if this system has not been initialization, construct radar rawTarAry arrays
        static std::vector<RadarTarget::Ptr> tarAry = {};

        tarAry.insert(tarAry.end(), rawTarAry->GetTargets().cbegin(), rawTarAry->GetTargets().cend());

        static double scanHeadTime = tarAry.front()->GetTimestamp();

        if (tarAry.back()->GetTimestamp() - scanHeadTime < 0.1) { return; }

        // if time span is larger than 0.1 (s), try to organize these targets as a radar frame
        if (tarAry.size() < 3) {
            // this radar frame is invalid, clear current status
            tarAry.clear();

            LOCK_RADAR_INIT_FRAMES
            radarTarAryForInit.clear();
            return;
        }

        // this radar frame is valid, compute average time
        double avgTime = 0.0;
        for (const auto &item: tarAry) { avgTime += item->GetTimestamp(); }
        avgTime /= static_cast<double>(tarAry.size());

        {
            LOCK_RADAR_INIT_FRAMES
            radarTarAryForInit.push_back(RadarTargetArray::Create(avgTime, tarAry));

            if (radarTarAryForInit.size() > 6) {
                // keep data that lasting for 1.0 (s), i.e., ten frames, each lasting 0.1 (s)
                radarTarAryForInit.pop_front();

                LOCK_TRAIL_STATUS
                TRaILStatus::DataManager::CurStatus |= TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady;
            }
        }

        // clear current status
        scanHeadTime = tarAry.back()->GetTimestamp();
        tarAry.clear();
    }

    std::list<RadarTargetArray::Ptr> DataManager::GetRadarTarAryForInitSafely() const {
        LOCK_RADAR_INIT_FRAMES
        return radarTarAryForInit;
    }

    std::list<IMUFrame::Ptr> DataManager::ExtractIMUDataPieceSafely(double start, double end) {
        LOCK_IMU_DATA_SEQ
        std::list<IMUFrame::Ptr> dataSeq;
        auto sIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [start](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < start;
        }).base();
        auto eIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [end](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < end;
        }).base();
        std::copy(sIter, eIter, std::back_inserter(dataSeq));
        return dataSeq;
    }

    std::list<IMUFrame::Ptr> DataManager::ExtractIMUDataPieceSafely(double start) {
        LOCK_IMU_DATA_SEQ
        std::list<IMUFrame::Ptr> dataSeq;
        auto sIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [start](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() < start;
        }).base();
        std::copy(sIter, imuDataSeq.end(), std::back_inserter(dataSeq));
        return dataSeq;
    }

    std::list<RadarTarget::Ptr> DataManager::ExtractRadarDataPieceSafely(double start, double end) {
        LOCK_RADAR_DATA_SEQ
        std::list<RadarTarget::Ptr> tarSeq;
        auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [start](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < start;
        }).base();
        auto eIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [end](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < end;
        }).base();
        std::copy(sIter, eIter, std::back_inserter(tarSeq));
        return tarSeq;
    }

    std::list<RadarTarget::Ptr> DataManager::ExtractRadarDataPieceSafely(double start) {
        LOCK_RADAR_DATA_SEQ
        std::list<RadarTarget::Ptr> tarSeq;

        auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [start](const RadarTarget::Ptr &tar) {
            return tar->GetTimestamp() < start;
        }).base();
        std::copy(sIter, radarDataSeq.end(), std::back_inserter(tarSeq));
        return tarSeq;
    }

    std::list<LidarTarget::Ptr> DataManager::ExtractLiDARDataPieceSafely(double start, double end) {
        LOCK_LIDAR_DATA_SEQ
        std::list<LidarTarget::Ptr> tarSeq;
        auto sIter = std::find_if(lidarDataSeq.rbegin(), lidarDataSeq.rend(), [start](const LidarTarget::Ptr &tar) {
            return tar->GetTimestamp().first < start;
        }).base();
        auto eIter = std::find_if(lidarDataSeq.rbegin(), lidarDataSeq.rend(), [end](const LidarTarget::Ptr &tar) {
            return tar->GetTimestamp().second < end;
        }).base();
        std::copy(sIter, eIter, std::back_inserter(tarSeq));
        return tarSeq;
    }

    std::list<LidarTarget::Ptr> DataManager::ExtractLiDARDataPieceSafely(double start) {
        LOCK_LIDAR_DATA_SEQ
        std::list<LidarTarget::Ptr> tarSeq;

        auto sIter = std::find_if(lidarDataSeq.rbegin(), lidarDataSeq.rend(), [start](const LidarTarget::Ptr &tar) {
            return tar->GetTimestamp().first < start;
        }).base();
        std::copy(sIter, lidarDataSeq.end(), std::back_inserter(tarSeq));
        return tarSeq;
    }

    double DataManager::GetEndTimeSafely() const {
        double m1, m2, m3;
        {
            LOCK_IMU_DATA_SEQ
            m1 = imuDataSeq.back()->GetTimestamp();
        }
        {
            LOCK_LIDAR_DATA_SEQ
            m2 = lidarDataSeq.back()->GetTimestamp().first;
        }
        double m = std::min({m1, m2});

        return m;
    }

    const std::optional<double> &DataManager::GetTRaILTimeEpoch() const {
        return TRAIL_TIME_EPOCH;
    }

    double DataManager::GetIMUEndTimeSafely() const {
        LOCK_IMU_DATA_SEQ
        return imuDataSeq.back()->GetTimestamp();
    }

    double DataManager::GetRadarEndTimeSafely() const {
        LOCK_RADAR_DATA_SEQ
        return radarDataSeq.back()->GetTimestamp();
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> DataManager::AcceMeanVar(const std::list<IMUFrame::Ptr> &data) {
        if (data.empty()) { return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()}; }

        Eigen::MatrixXd matrix(data.size(), 3);
        int i = 0;
        for (const auto &v: data) { matrix.row(i++) = v->GetAcce(); }

        Eigen::Vector3d mean = matrix.colwise().mean();
        Eigen::Matrix3d var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                               (matrix.rowwise() - matrix.colwise().mean())) / static_cast<double>(data.size() - 1);

        return {mean, var};
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> DataManager::GyroMeanVar(const std::list<IMUFrame::Ptr> &data) {
        if (data.empty()) { return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()}; }

        Eigen::MatrixXd matrix(data.size(), 3);
        int i = 0;
        for (const auto &v: data) { matrix.row(i++) = v->GetGyro(); }

        Eigen::Vector3d mean = matrix.colwise().mean();
        Eigen::Matrix3d var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                               (matrix.rowwise() - matrix.colwise().mean())) / static_cast<double>(data.size() - 1);

        return {mean, var};
    }

    void DataManager::EraseOldDataPieceSafely(double time) {
        {
            LOCK_IMU_DATA_SEQ
            auto iter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [time](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < time;
            }).base();
            imuDataSeq.erase(imuDataSeq.begin(), iter);
        }
        {
            LOCK_RADAR_DATA_SEQ
            auto iter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [time](const RadarTarget::Ptr &tar) {
                return tar->GetTimestamp() < time;
            }).base();
            radarDataSeq.erase(radarDataSeq.begin(), iter);
        }
        {
            LOCK_LIDAR_DATA_SEQ
            auto iter = std::find_if(lidarDataSeq.rbegin(), lidarDataSeq.rend(), [time](const LidarTarget::Ptr &tar) {
                return tar->GetTimestamp().second < time;
            }).base();
            lidarDataSeq.erase(lidarDataSeq.begin(), iter);
        }
    }

    const std::list<IMUFrame::Ptr> &DataManager::GetIMUDataSeq() const {
        return imuDataSeq;
    }

    const std::list<RadarTarget::Ptr> &DataManager::GetRadarDataSeq() const {
        return radarDataSeq;
    }

    const std::list<LidarTarget::Ptr> &DataManager::GetLidarDataSeq() const {
        return lidarDataSeq;
    }

}
