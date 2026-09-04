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

#ifndef TRAIL_DATA_MANAGER_H
#define TRAIL_DATA_MANAGER_H

#include "config/configor.h"
#include "sensor/imu_data_loader.h"
#include "sensor/radar_data_loader.h"
#include "sensor/lidar_data_loader.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "core/status.h"


#include <functional>

using namespace std::placeholders;

namespace trail {
    // unique locks for mutexes
#define LOCK_IMU_DATA_SEQ std::unique_lock<std::mutex> imuDataLock(DataManager::IMUDataSeqMutex);
#define LOCK_RADAR_DATA_SEQ std::unique_lock<std::mutex> radarDataLock(DataManager::RadarDataSeqMutex);
#define LOCK_LIDAR_DATA_SEQ std::unique_lock<std::mutex> lidarDataLock(DataManager::LidarDataSeqMutex);
#define LOCK_RADAR_INIT_FRAMES std::unique_lock<std::mutex> radarInitFramesLock(DataManager::RadarInitFramesMutex);

    class DataManager {
    public:
        using Ptr = std::shared_ptr<DataManager>;

    private:
        // ros node handler used for creating subscribers, i.e., 'imuSuber' and 'radarSuber'
        rclcpp::Node::SharedPtr handler;
        Configor::Ptr configor;

        // ros subscribers to receive imu and radar data
        rclcpp::SubscriptionBase::SharedPtr imuSuber;
        rclcpp::SubscriptionBase::SharedPtr radarSuber;
        rclcpp::SubscriptionBase::SharedPtr lidarSuber;

        // containers to store sensor data
        std::list<IMUFrame::Ptr> imuDataSeq;
        std::list<RadarTarget::Ptr> radarDataSeq;
        std::list<LidarTarget::Ptr> lidarDataSeq;

        // the radar target array for initialization, size: 10
        std::list<RadarTargetArray::Ptr> radarTarAryForInit;

        std::optional<double> TRAIL_TIME_EPOCH;

        bool contact=false;
        int contact_duration = 0;

        bool imu_is_come = false;
        Eigen::Quaternion<double> init_orient = Eigen::Quaternion<double>::Identity();
        Eigen::Quaternion<double> prev_orient = Eigen::Quaternion<double>::Identity();
        Eigen::Quaternion<double> SO3_W2Ref = Eigen::Quaternion<double>::Identity();
        

    public:
        // mutexes employed in multi-thread framework
        static std::mutex IMUDataSeqMutex;
        static std::mutex RadarDataSeqMutex;
        static std::mutex LidarDataSeqMutex;
        static std::mutex RadarInitFramesMutex;

    public:
        explicit DataManager(const rclcpp::Node::SharedPtr &handler, const Configor::Ptr &configor);

        static Ptr Create(const rclcpp::Node::SharedPtr &handler, const Configor::Ptr &configor);

        [[nodiscard]] const std::optional<double> &GetTRaILTimeEpoch() const;

        [[nodiscard]] double GetEndTimeSafely() const;

        [[nodiscard]] double GetIMUEndTimeSafely() const;

        [[nodiscard]] double GetRadarEndTimeSafely() const;

        const std::list<IMUFrame::Ptr> &GetIMUDataSeq() const;

        const std::list<RadarTarget::Ptr> &GetRadarDataSeq() const;

        const std::list<LidarTarget::Ptr> &GetLidarDataSeq() const;

        void ShowDataStatus() const;

        [[nodiscard]] std::list<RadarTargetArray::Ptr> GetRadarTarAryForInitSafely() const;

        std::list<IMUFrame::Ptr> ExtractIMUDataPieceSafely(double start, double end);

        std::list<IMUFrame::Ptr> ExtractIMUDataPieceSafely(double start);

        std::list<RadarTarget::Ptr> ExtractRadarDataPieceSafely(double start, double end);

        std::list<RadarTarget::Ptr> ExtractRadarDataPieceSafely(double start);

        std::list<LidarTarget::Ptr> ExtractLiDARDataPieceSafely(double start, double end);

        std::list<LidarTarget::Ptr> ExtractLiDARDataPieceSafely(double start);

        void LidarPointTimeCorrection(const LidarTarget::Ptr &lidar_scan);

        static std::pair<Eigen::Vector3d, Eigen::Matrix3d> AcceMeanVar(const std::list<IMUFrame::Ptr> &data);

        static std::pair<Eigen::Vector3d, Eigen::Matrix3d> GyroMeanVar(const std::list<IMUFrame::Ptr> &data);

        void EraseOldDataPieceSafely(double time);

    protected:
        IMUFrame::Ptr ApplySMA(const IMUFrame::Ptr& frame) {

            // smoothedFrame.header = frame->header;
            Eigen::Vector3d sumLinearAccel;
            sumLinearAccel.x() = 0.0;
            sumLinearAccel.y() = 0.0;
            sumLinearAccel.z() = 0.0;
            size_t count = 0;

            auto it = imuDataSeq.rbegin();
            for (; it != imuDataSeq.rend() && count < 9; ++it) {
                const auto& prevFrame = *it;
                Eigen::Vector3d temp = prevFrame->GetAcce();
                sumLinearAccel.x() += temp.x();
                sumLinearAccel.y() += temp.y();
                sumLinearAccel.z() += temp.z();
                count++;
            }

            Eigen::Vector3d temp = frame->GetAcce();
            sumLinearAccel.x() += temp.x();
            sumLinearAccel.y() += temp.y();
            sumLinearAccel.z() += temp.z();
            
            count++;

            sumLinearAccel.x() = sumLinearAccel.x() / count;
            sumLinearAccel.y() = sumLinearAccel.y() / count;
            sumLinearAccel.z() = sumLinearAccel.z() / count;
            
            return IMUFrame::Create(frame->GetTimestamp(), frame->GetGyro(), sumLinearAccel, frame->GetOrientation());
        }

        // template<class IMUMsgType>
        // void HandleIMUMessage(const typename IMUMsgType::ConstPtr &msg) {
        //     auto frame = IMUDataUnpacker::Unpack(msg);
        //     if(!imu_is_come){
        //         imu_is_come = true;
        //         prev_orient = frame->GetOrientation(); 
        //         SO3_W2Ref = frame->GetOrientation(); //R_RefW
        //     }
        //     else{
        //         Eigen::Matrix3d R_wo = init_orient.toRotationMatrix();
                
        //         Eigen::Matrix3d R_wc = frame->GetOrientation().toRotationMatrix();
        //         Eigen::Matrix3d delta_R = prev_orient.toRotationMatrix().inverse() * R_wc;
        //         Eigen::Vector3d gyro = frame->GetGyro();
        //         double dt = frame->GetTimestamp() - imuDataSeq.back()->GetTimestamp() - *TRAIL_TIME_EPOCH;

        //         Eigen::AngleAxisd rollAngle(gyro(0)*dt, Eigen::Vector3d::UnitX());   
        //         Eigen::AngleAxisd pitchAngle(gyro(1)*dt, Eigen::Vector3d::UnitY());
        //         Eigen::AngleAxisd yawAngle(gyro(2)*dt, Eigen::Vector3d::UnitZ()); 

        //         Eigen::Matrix3d R_no_yaw = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

        //         Eigen::Matrix3d R_oc = R_wo * R_no_yaw;
        //         double pitch = atan2(-delta_R(2, 0), sqrt(delta_R(2, 1) * delta_R(2, 1) + delta_R(2, 2) * delta_R(2, 2))); // theta
        //         double roll = atan2(delta_R(2, 1), delta_R(2, 2));          // phi
        //         double yaw = atan2(R_oc(1, 0), R_oc(0, 0));           // psi

        //         Eigen::AngleAxisd rollAngle_angular(roll, Eigen::Vector3d::UnitX());   
        //         Eigen::AngleAxisd pitchAngle_angular(pitch, Eigen::Vector3d::UnitY()); 
        //         Eigen::AngleAxisd yawAngle_angular(yaw, Eigen::Vector3d::UnitZ());    

        //         Eigen::Matrix3d R_refine = (yawAngle_angular * pitchAngle_angular * rollAngle_angular).toRotationMatrix();

        //         init_orient = Eigen::Quaternion<double>(R_refine); 
        //         // prev_orient = frame->GetOrientation(); 
                
        //     }

        //     Eigen::Quaternion<double> relative_orientation = Eigen::Quaternion<double>(init_orient);

        //     frame->SetOrientation(relative_orientation);
        //     // try to initialize the time epoch of TRaIL
        //     if (TRAIL_TIME_EPOCH == std::nullopt) {
        //         TRAIL_TIME_EPOCH = frame->GetTimestamp();
        //         spdlog::info("time epoch of TRaIL initialized by imu frame: {:.6f}", *TRAIL_TIME_EPOCH);
        //     }
        //     frame->SetTimestamp(frame->GetTimestamp() - *TRAIL_TIME_EPOCH);
        //     {
        //         LOCK_IMU_DATA_SEQ
        //         auto smoothedFrame = ApplySMA(frame);
        //         // imuDataSeq.push_back(smoothedFrame); 
        //         if(!imuDataSeq.empty()){
        //             if(contact_duration>2){
        //                 imuDataSeq.push_back(smoothedFrame); 
        //             }
        //             else{
        //                 auto frame_ = IMUFrame::Create(frame->GetTimestamp(), frame->GetGyro(), imuDataSeq.back()->GetAcce(), frame->GetOrientation());
        //                 imuDataSeq.push_back(frame_); 
        //             }
        //         }
        //         else{
        //             imuDataSeq.push_back(frame); 
        //         }
        //         // imuDataSeq.push_back(frame); 
        //     }
            
        // }
        template<class IMUMsgType>
        void HandleIMUMessage(const typename IMUMsgType::ConstPtr &msg) {
            auto frame = IMUDataUnpacker::Unpack(msg, configor->preprocess.imu_acc_scale);
            // try to initialize the time epoch of TRaIL
            if (TRAIL_TIME_EPOCH == std::nullopt) {
                TRAIL_TIME_EPOCH = frame->GetTimestamp();
                spdlog::info("time epoch of TRaIL initialized by imu frame: {:.6f}", *TRAIL_TIME_EPOCH);
            }
            frame->SetTimestamp(frame->GetTimestamp() - *TRAIL_TIME_EPOCH);
            const double t = frame->GetTimestamp();

            {
                LOCK_IMU_DATA_SEQ
                imuDataSeq.push_back(frame);
            }
        }

        template<class RadarMsgType>
        void HandleRadarMessage(const typename RadarMsgType::ConstPtr &msg) {
            auto targetAry = RadarDataUnpacker::Unpack(msg);
            // try to initialize the time epoch of TRaIL
            if (TRAIL_TIME_EPOCH == std::nullopt) {
                TRAIL_TIME_EPOCH = targetAry->GetTimestamp();
                spdlog::info("time epoch of TRaIL initialized by radar targetAry: {:.6f}", *TRAIL_TIME_EPOCH);
            }
            // aligned the time of targetAry (i.e., radar time) to IMU time
            targetAry->SetTimestamp(
                    targetAry->GetTimestamp() - *TRAIL_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
            );
            for (auto &tar: targetAry->GetTargets()) {
                tar->SetTimestamp(
                        tar->GetTimestamp() - *TRAIL_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
                );
            }
            {
                LOCK_RADAR_DATA_SEQ
                radarDataSeq.insert(
                        radarDataSeq.end(), targetAry->GetTargets().cbegin(), targetAry->GetTargets().cend()
                );
            }
            auto status = TRaILStatus::GetStatusPackSafely();
            if (!TRaILStatus::IsWith(TRaILStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                OrganizeRadarTarAryForInit(targetAry);
            } else {
                {
                    LOCK_RADAR_INIT_FRAMES
                    radarTarAryForInit.clear();
                }
                if (TRaILStatus::IsWith(TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr)) {
                    LOCK_TRAIL_STATUS
                    TRaILStatus::DataManager::CurStatus ^= TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady;
                }
            }
        }

        template<class RadarMsgType>
        void HandleRadarMessage_colo(const typename RadarMsgType::ConstPtr &msg) {
            auto targetAry = RadarDataUnpacker::Unpack_colo(msg);
            // try to initialize the time epoch of TRaIL
            if (TRAIL_TIME_EPOCH == std::nullopt) {
                TRAIL_TIME_EPOCH = targetAry->GetTimestamp();
                spdlog::info("time epoch of TRaIL initialized by radar targetAry: {:.6f}", *TRAIL_TIME_EPOCH);
            }
            // aligned the time of targetAry (i.e., radar time) to IMU time
            targetAry->SetTimestamp(
                    targetAry->GetTimestamp() - *TRAIL_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
            );
            for (auto &tar: targetAry->GetTargets()) {
                tar->SetTimestamp(
                        tar->GetTimestamp() - *TRAIL_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
                );
            }
            {
                LOCK_RADAR_DATA_SEQ
                radarDataSeq.insert(
                        radarDataSeq.end(), targetAry->GetTargets().cbegin(), targetAry->GetTargets().cend()
                );
            }
            auto status = TRaILStatus::GetStatusPackSafely();
            if (!TRaILStatus::IsWith(TRaILStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                OrganizeRadarTarAryForInit(targetAry);
            } else {
                {
                    LOCK_RADAR_INIT_FRAMES
                    radarTarAryForInit.clear();
                }
                if (TRaILStatus::IsWith(TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr)) {
                    LOCK_TRAIL_STATUS
                    TRaILStatus::DataManager::CurStatus ^= TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady;
                }
            }
        }

        template<class RadarMsgType>
        void HandleRadarMessage_coral(const typename RadarMsgType::ConstPtr &msg) {
            auto targetAry = RadarDataUnpacker::Unpack_coral(msg);
            // try to initialize the time epoch of TRaIL
            if (TRAIL_TIME_EPOCH == std::nullopt) {
                TRAIL_TIME_EPOCH = targetAry->GetTimestamp();
                spdlog::info("time epoch of TRaIL initialized by radar targetAry: {:.6f}", *TRAIL_TIME_EPOCH);
            }
            // aligned the time of targetAry (i.e., radar time) to IMU time
            targetAry->SetTimestamp(
                    targetAry->GetTimestamp() - *TRAIL_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
            );
            for (auto &tar: targetAry->GetTargets()) {
                tar->SetTimestamp(
                        tar->GetTimestamp() - *TRAIL_TIME_EPOCH + configor->dataStream.CalibParam.TIME_OFFSET_RtoB
                );
            }
            {
                LOCK_RADAR_DATA_SEQ
                radarDataSeq.insert(
                        radarDataSeq.end(), targetAry->GetTargets().cbegin(), targetAry->GetTargets().cend()
                );
            }
            auto status = TRaILStatus::GetStatusPackSafely();
            if (!TRaILStatus::IsWith(TRaILStatus::StateManager::Status::HasInitialized, status.StateMagr)) {
                OrganizeRadarTarAryForInit(targetAry);
            } else {
                {
                    LOCK_RADAR_INIT_FRAMES
                    radarTarAryForInit.clear();
                }
                if (TRaILStatus::IsWith(TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr)) {
                    LOCK_TRAIL_STATUS
                    TRaILStatus::DataManager::CurStatus ^= TRaILStatus::DataManager::Status::RadarTarAryForInitIsReady;
                }
            }
        }

        template<class LidarMsgType>
        void HandleLidarMessage(const typename LidarMsgType::ConstPtr &msg) {
            auto lidar_scan = LidarDataUnpacker::Unpack(msg, configor->preprocess.timestamp_unit, configor->preprocess.point_filter_num, configor->preprocess.blind);
            // // try to initialize the time epoch of TRaIL
            if (TRAIL_TIME_EPOCH == std::nullopt) {
                TRAIL_TIME_EPOCH = lidar_scan->GetTimestamp().first;
                spdlog::info("time epoch of TRaIL initialized by lidar targetAry: {:.6f}", *TRAIL_TIME_EPOCH);
            }
            auto [t0, t1] = lidar_scan->GetTimestamp();
            lidar_scan->SetTimestamp(
                { t0 - *TRAIL_TIME_EPOCH, t1 - *TRAIL_TIME_EPOCH }
            );
            LidarPointTimeCorrection(lidar_scan);
            
            {
                LOCK_LIDAR_DATA_SEQ
                lidarDataSeq.push_back(lidar_scan); // please uncomment this line if you want to store the lidar data
            }
        }

        template<class LidarMsgType>
        void HandleLidarMessage_MID(const typename LidarMsgType::ConstPtr &msg) {
            auto lidar_scan = LidarDataUnpacker::Unpack_MID(msg, configor->preprocess.timestamp_unit, configor->preprocess.point_filter_num, configor->preprocess.blind);
            // // try to initialize the time epoch of TRaIL
            if (TRAIL_TIME_EPOCH == std::nullopt) {
                TRAIL_TIME_EPOCH = lidar_scan->GetTimestamp().first;
                spdlog::info("time epoch of TRaIL initialized by lidar targetAry: {:.6f}", *TRAIL_TIME_EPOCH);
            }
            auto [t0, t1] = lidar_scan->GetTimestamp();
            lidar_scan->SetTimestamp(
                { t0 - *TRAIL_TIME_EPOCH, t1 - *TRAIL_TIME_EPOCH }
            );
            LidarPointTimeCorrection(lidar_scan);

            {
                LOCK_LIDAR_DATA_SEQ
                lidarDataSeq.push_back(lidar_scan); // please uncomment this line if you want to store the lidar data
            }
        }

        template<class LidarMsgType>
        void HandleLidarMessage_RAI(const typename LidarMsgType::ConstPtr &msg) {
            auto lidar_scan = LidarDataUnpacker::Unpack_RAI(msg, configor->preprocess.timestamp_unit, configor->preprocess.point_filter_num, configor->preprocess.blind);
            // // try to initialize the time epoch of TRaIL
            if (TRAIL_TIME_EPOCH == std::nullopt) {
                TRAIL_TIME_EPOCH = lidar_scan->GetTimestamp().first;
                spdlog::info("time epoch of TRaIL initialized by lidar targetAry: {:.6f}", *TRAIL_TIME_EPOCH);
            }
            auto [t0, t1] = lidar_scan->GetTimestamp();
            lidar_scan->SetTimestamp(
                { t0 - *TRAIL_TIME_EPOCH, t1 - *TRAIL_TIME_EPOCH }
            );
            LidarPointTimeCorrection(lidar_scan);

            {
                LOCK_LIDAR_DATA_SEQ
                lidarDataSeq.push_back(lidar_scan); // please uncomment this line if you want to store the lidar data
            }
        }

        template<class LidarMsgType>
        void HandleLidarMessage_HESAI(const typename LidarMsgType::ConstPtr &msg) {
            auto lidar_scan = LidarDataUnpacker::Unpack_HESAI(msg, configor->preprocess.timestamp_unit, configor->preprocess.point_filter_num, configor->preprocess.blind);
            // // try to initialize the time epoch of TRaIL
            if (TRAIL_TIME_EPOCH == std::nullopt) {
                TRAIL_TIME_EPOCH = lidar_scan->GetTimestamp().first;
                spdlog::info("time epoch of TRaIL initialized by lidar targetAry: {:.6f}", *TRAIL_TIME_EPOCH);
            }
            auto [t0, t1] = lidar_scan->GetTimestamp();
            lidar_scan->SetTimestamp(
                { t0 - *TRAIL_TIME_EPOCH, t1 - *TRAIL_TIME_EPOCH }
            );
            LidarPointTimeCorrection(lidar_scan);

            {
                LOCK_LIDAR_DATA_SEQ
                lidarDataSeq.push_back(lidar_scan); // please uncomment this line if you want to store the lidar data
            }
        }

        inline void OrganizeRadarTarAryForInit(const RadarTargetArray::Ptr &rawTarAry);
    };

}


#endif //TRAIL_DATA_MANAGER_H
