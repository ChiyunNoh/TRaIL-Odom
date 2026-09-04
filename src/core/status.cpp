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


#include "core/status.h"

namespace trail {
    // ------------------------
    // static initialized filed
    // ------------------------
    std::mutex TRaILStatus::StatusMutex = {};

    TRaILStatus::StateManager::Status TRaILStatus::StateManager::CurStatus = TRaILStatus::StateManager::Status::NONE;
    double TRaILStatus::StateManager::ValidStateEndTime = -1.0;

    TRaILStatus::DataManager::Status TRaILStatus::DataManager::CurStatus = TRaILStatus::DataManager::Status::NONE;

    TRaILStatus::StatusPack::StatusPack(TRaILStatus::StateManager::Status stateMagr, double ValidStateEndTime,
                                        TRaILStatus::DataManager::Status dataMagr)
            : StateMagr(stateMagr), ValidStateEndTime(ValidStateEndTime), DataMagr(dataMagr) {}

    TRaILStatus::StatusPack TRaILStatus::GetStatusPackSafely() {
        LOCK_TRAIL_STATUS
        return {StateManager::CurStatus, StateManager::ValidStateEndTime, DataManager::CurStatus};
    }
}