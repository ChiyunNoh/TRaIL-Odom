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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "thread"
#include "core/status.h"
#include "core/trail.h"

// config the 'spdlog' log pattern
void ConfigSpdlog() {
    // [log type]-[thread]-[time] message
    spdlog::set_pattern("%^[%L]%$-[%t]-[%H:%M:%S.%e] %v");

    // set log level
    spdlog::set_level(spdlog::level::warn);
}

void PrintLibInfo() {
    std::cout << "TRaIL-Odom\n"
                 "Tightly Coupled Continuous Time Radar-IMU-LiDAR Odometry\n"
                 "with Adaptive Doppler Weighting\n"
                 "Authors: Chiyun Noh, Turcan Tuna, William Talbot, Marco Hutter,\n"
                 "         Laurent Kneip, and Ayoung Kim"
              << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        ConfigSpdlog();

        PrintLibInfo();

        // obtain the path of config file
        auto node = rclcpp::Node::make_shared("trail_node");

        node->declare_parameter<std::string>("config_path", "");
        std::string configPath = node->get_parameter("config_path").as_string();

        if (configPath.empty()) {
            RCLCPP_FATAL(node->get_logger(),
                        "Parameter '/trail_node/config_path' not set -- "
                        "run with:  ros2 run trail trail_node "
                        " --ros-args -p config_path:=/absolute/path/to/trail.yaml");
            rclcpp::shutdown();
            return 1;
        }
        spdlog::info("loading configure from json file '{}'...", configPath);

        // load the configure file
        auto configor = trail::Configor::Load(configPath);
        configor->PrintMainFields();

        // create 'TRaIL-Odom'
        auto trail = trail::TRaIL::Create(configor);

        // perform solving
        trail->Run();

        // save results
        trail->Save();

    } catch (const trail::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case trail::Status::Flag::FINE:
                // this case usually won't happen
                spdlog::info(status.what);
                break;
            case trail::Status::Flag::WARNING:
                spdlog::warn(status.what);
                break;
            case trail::Status::Flag::ERROR:
                spdlog::error(status.what);
                break;
            case trail::Status::Flag::CRITICAL:
                spdlog::critical(status.what);
                break;
        }
    } catch (const std::exception &e) {
        // an unknown exception not thrown by this program
        spdlog::critical(e.what());
    }
    rclcpp::shutdown();
    return 0;
}
