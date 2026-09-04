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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include "spdlog/spdlog.h"
#include <Eigen/Eigenvalues>


#include <rms/histogram.h>

namespace rms
{

/* //{ class RMS */

using t_points  = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using t_voxel   = Eigen::Vector3i;

class RMS {

  // | ---------------------- Voxelization ---------------------- |
  struct VoxelHash
  {
    size_t operator()(const t_voxel& voxel) const {
      const std::uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
      return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
  };

  // | ----------------------- Public API ----------------------- |
public:
  Eigen::Matrix3f Htt = Eigen::Matrix3f::Zero();
  RMS(size_t K, float lambda, float voxel_input, float voxel_output);
  pcl::PointCloud<pcl::PointXYZ>::Ptr vectorEigenToPCL(const std::vector<Eigen::Vector3d> &frame);
  t_indices sample(std::vector<Eigen::Vector3d> &frame, std::vector<double> &timestamps);

  // | --------------- ROS and conversion methods --------------- |
private:
  void extractByIndices(std::vector<Eigen::Vector3d> &frame,
                        std::vector<double> &timestamps,
                        const t_indices& indices_in);

  // | ---------------- RMS variables and methods --------------- |
private:
  size_t _K            = 10;
  float  _lambda       = 1.0f;
  float  _voxel_input  = -1.0f;
  float  _voxel_output = -1.0f;

  void voxelizeIndices(const float voxel_size, const t_points pc_in, t_indices& indices_inout, const bool check_NaNs = true);

  std::vector<t_gfh> computeGFH(const t_points points, const t_indices& indices);
  t_indices          sampleByGFH(const t_points points, const std::vector<t_gfh>& gfh, const size_t K, const float lambda);
};

//}

}  // namespace rms
