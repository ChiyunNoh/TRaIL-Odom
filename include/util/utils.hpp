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

#ifndef TRAIL_UTILS_HPP
#define TRAIL_UTILS_HPP

#include <ceres/jet.h>
#include "ctraj/utils/sophus_utils.hpp"
#include "spdlog/fmt/fmt.h"
#include "filesystem"
#include "util/cereal_archive_helper.hpp"
#include "ctraj/core/pose.hpp"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZIN;

namespace trail {
#define TRAIL_TO_STR(obj)         \
    [&]() -> std::string {        \
        std::stringstream stream; \
        stream << obj;            \
        return stream.str();      \
    }()

    template<typename ScaleType>
    inline std::string FormatValueVector(const std::vector<const char *> &descVec,
                                         const std::vector<ScaleType> &valVec,
                                         const char *scaleFormatStr = "{:+011.6f}") {
        std::string str;
        const int M = static_cast<int>(descVec.size());
        for (int i = 0; i < (M - 1); ++i) {
            str += '\'' + std::string(descVec.at(i)) + "': " +
                   fmt::format(scaleFormatStr, valVec.at(i)) + ", ";
        }
        str += '\'' + std::string(descVec.at(M - 1)) + "': " +
               fmt::format(scaleFormatStr, valVec.at(M - 1));
        return str;
    }

    /**
     * @brief a function to get all the filenames in the directory
     * @param directory the directory
     * @return the filenames in the directory
     */
    inline std::vector<std::string> FilesInDir(const std::string &directory) {
        std::vector<std::string> files;
        for (const auto &elem: std::filesystem::directory_iterator(directory))
            if (elem.status().type() != std::filesystem::file_type::directory)
                files.emplace_back(std::filesystem::canonical(elem.path()).c_str());
        std::sort(files.begin(), files.end());
        return files;
    }

    /**
     * @brief a function to split a string to some string elements according the splitor
     * @param str the string to be split
     * @param splitor the splitor char
     * @param ignoreEmpty whether ignoring the empty string element or not
     * @return the split string vector
     */
    inline std::vector<std::string> SplitString(const std::string &str, char splitor, bool ignoreEmpty = true) {
        std::vector<std::string> vec;
        auto iter = str.cbegin();
        while (true) {
            auto pos = std::find(iter, str.cend(), splitor);
            auto elem = std::string(iter, pos);
            if (!(elem.empty() && ignoreEmpty)) {
                vec.push_back(elem);
            }
            if (pos == str.cend()) {
                break;
            }
            iter = ++pos;
        }
        return vec;
    }

    // template <typename T>
    // inline Eigen::Matrix<T, 3, 3> skew_sym(const Eigen::Matrix<T, 3, 1>& v) {
    //     Eigen::Matrix<T, 3, 3> hat;
    //     hat << 0, -v(2), v(1),
    //         v(2), 0, -v(0),
    //         -v(1), v(0), 0;
    //     return hat;
    // }

    // template <typename T>
    // inline Eigen::Matrix<T, 3, 3> Rodrigues(const Eigen::Matrix<T, 3, 1>& direction) {
    //     Eigen::Matrix<T, 3, 1> axis(T(0), T(0), T(1));
    //     Eigen::Matrix<T, 3, 1> cross = skew_sym(axis) * direction;
    //     Eigen::Matrix<T, 3, 3> rotation = Eigen::Matrix<T, 3, 3>::Identity()
    //                                     + skew_sym(cross)
    //                                     + (T(1) / (T(1) + axis.dot(direction))) * skew_sym(cross) * skew_sym(cross);
    //     return rotation;
    // }

    // template <typename T>
    // inline Eigen::Matrix<T, 2, 1> Logarithm(const Eigen::Matrix<T, 3, 1>& direction) {
    //     Eigen::Matrix<T, 3, 1> axis(T(0), T(0), T(1));
    //     Eigen::Matrix<T, 3, 2> B;
    //     B << T(0), T(0),
    //         T(1), T(0),
    //         T(0), T(1);

    //     T cos_theta = axis.dot(direction);
    //     T theta = ceres::acos(cos_theta);

    //     // Prevent division by zero
    //     T norm_factor = (B.transpose() * direction).norm();
    //     if (norm_factor < T(1e-8)) {
    //         return Eigen::Matrix<T, 2, 1>::Zero();
    //     }

    //     Eigen::Matrix<T, 2, 1> log = theta * B.transpose() * direction / norm_factor;

    //     return log;
    // }

    template<typename Scale, int Rows, int Cols>
    inline Eigen::Matrix<Scale, Rows, Cols>
    TrapIntegrationOnce(const std::vector<std::pair<Scale, Eigen::Matrix<Scale, Rows, Cols>>> &data) {
        Eigen::Matrix<Scale, Rows, Cols> sum = Eigen::Matrix<Scale, Rows, Cols>::Zero();
        for (int i = 0; i < static_cast<int>(data.size()) - 1; ++i) {
            int j = i + 1;
            const auto &di = data.at(i);
            const auto &dj = data.at(j);
            sum += (di.second + dj.second) * (dj.first - di.first) * Scale(0.5);
        }
        return sum;
    }

    template<typename Scale, int Rows, int Cols>
    inline Eigen::Matrix<Scale, Rows, Cols>
    TrapIntegrationTwice(const std::vector<std::pair<Scale, Eigen::Matrix<Scale, Rows, Cols>>> &data) {
        std::vector<std::pair<Scale, Eigen::Matrix<Scale, Rows, Cols>>> dataOnce;
        Eigen::Matrix<Scale, Rows, Cols> sum = Eigen::Matrix<Scale, Rows, Cols>::Zero();
        for (int i = 0; i < data.size() - 1; ++i) {
            int j = i + 1;
            const auto &di = data.at(i);
            const auto &dj = data.at(j);
            sum += (di.second + dj.second) * (dj.first - di.first) * Scale(0.5);
            dataOnce.push_back({(dj.first + di.first) * Scale(0.5), sum});
        }
        return TrapIntegrationOnce(dataOnce);
    }

    template<typename EigenVectorType>
    inline auto EigenVecXToVector(const EigenVectorType &eigenVec) {
        std::vector<typename EigenVectorType::Scalar> vec(eigenVec.rows());
        for (int i = 0; i < static_cast<int>(vec.size()); ++i) {
            vec.at(i) = eigenVec(i);
        }
        return vec;
    }

    template<class ScaleType>
    inline Sophus::SO3<ScaleType> ComputeKarcherMean(const std::vector<Sophus::SO3<ScaleType>> &so3Vec,
                                                     double tolerance = 1E-15) {
        if (so3Vec.empty()) {
            return {};
        }
        Sophus::SO3<ScaleType> X = so3Vec.front();
        while (true) {
            Eigen::Vector3<ScaleType> A = Eigen::Vector3<ScaleType>::Zero();
            for (const auto &item: so3Vec) {
                A += (X.inverse() * item).log();
            }
            A /= static_cast<double>(so3Vec.size());
            if (A.norm() < tolerance) {
                break;
            } else {
                X = X * Sophus::SO3<ScaleType>::exp(A);
            }
        }
        return X;
    }

    template<class Scale, int Rows, int Cols>
    inline Eigen::Matrix<Scale, Rows, Cols>
    ComputeMatVecMean(const std::vector<Eigen::Matrix<Scale, Rows, Cols>> &vec) {
        Eigen::Matrix<Scale, Rows, Cols> X = Eigen::Matrix<Scale, Rows, Cols>::Zero();
        for (const auto &item: vec) {
            X += item;
        }
        X /= static_cast<double>(vec.size());
        return X;
    }

    template<class Type>
    inline Type ComputeNumericalMean(const std::vector<Type> &vec) {
        Type X = static_cast<Type>(double{0.0});
        for (const auto &item: vec) {
            X += item;
        }
        X /= static_cast<double >(vec.size());
        return X;
    }

    template<class KeyType, class ValueType>
    inline std::vector<ValueType> ValueVecFromMap(const std::map<KeyType, ValueType> &m) {
        std::vector<ValueType> v;
        std::transform(m.begin(), m.end(), std::back_inserter(v), [](const std::pair<KeyType, ValueType> &p) {
            return p.second;
        });
        return v;
    }

    inline bool SavePoseSequence(const Eigen::aligned_vector<ns_ctraj::Posed> &poseSeq, const std::string &filename,
                                 CerealArchiveType::Enum archiveType) {
        if (auto parPath = std::filesystem::path(filename).parent_path();
                !exists(parPath) && !std::filesystem::create_directories(parPath)) {
            return false;
        }
        std::ofstream file(filename);
        auto ar = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("pose_seq", poseSeq));
        return true;
    }

    inline std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloudXYZIN::Ptr &_scan)
    {
        std::vector<Eigen::Vector3d> points;
        points.reserve(_scan ? _scan->size() : 0);

        if (!_scan) return points;

        for (const auto &p : _scan->points) {
            points.emplace_back(p.x, p.y, p.z);
        }
        return points;
    }

    inline std::vector<double> GetTimestamps(const PointCloudXYZIN::Ptr &_scan)
    {
        std::vector<double> ts;
        if (!_scan) return ts;

        ts.reserve(_scan->size());
        for (const auto &p : _scan->points) {
            // Common convention: relative time in curvature
            ts.emplace_back(static_cast<double>(p.curvature));
        }
        return ts;
    }

}

#endif //TRAIL_UTILS_HPP
