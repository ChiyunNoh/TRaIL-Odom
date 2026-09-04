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

#ifndef TRAIL_ENUM_CAST_HPP
#define TRAIL_ENUM_CAST_HPP

#include "exception"
#include <magic_enum.hpp>
#include "string"
#include <stdexcept>

namespace trail {

    struct EnumCast {
        template<class EnumType>
        static constexpr auto stringToEnum(const std::string &enumStr) {
            if (auto color = magic_enum::enum_cast<EnumType>(enumStr);color.has_value()) {
                return color.value();
            } else {
                throw std::runtime_error("'EnumCast::stringToEnum' cast failed");
            }
        }

        template<class EnumType>
        static constexpr auto integerToEnum(int enumValue) {
            if (auto color = magic_enum::enum_cast<EnumType>(enumValue);color.has_value()) {
                return color.value();
            } else {
                throw std::runtime_error("'EnumCast::integerToEnum' cast failed");
            }
        }

        template<class EnumType>
        static constexpr auto enumToInteger(EnumType enumType) {
            return magic_enum::enum_integer(enumType);
        }

        template<class EnumType>
        static constexpr auto enumToString(EnumType enumType) {
            return magic_enum::enum_name(enumType);
        }

        template<class EnumType>
        static constexpr auto stringToInteger(const std::string &enumStr) {
            return enumToInteger(stringToEnum<EnumType>(enumStr));
        }

        template<class EnumType>
        static constexpr auto integerToString(int enumValue) {
            return enumToString(integerToEnum<EnumType>(enumValue));
        }
    };

}

#endif //TRAIL_ENUM_CAST_HPP
