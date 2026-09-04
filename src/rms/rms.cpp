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

#include "rms/rms.h"
#include <tsl/robin_map.h>  // Include here since it's a header-only library

namespace rms
{

/*//{ RMS() constructor */
RMS::RMS(size_t K, float lambda, float voxel_input, float voxel_output)
    : _K(K), _lambda(lambda), _voxel_input(voxel_input), _voxel_output(voxel_output) {}

/*//}*/

pcl::PointCloud<pcl::PointXYZ>::Ptr
RMS::vectorEigenToPCL(const std::vector<Eigen::Vector3d> &frame)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    cloud->points.reserve(frame.size());

    for (const auto &p : frame) {
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(p.x());
        pt.y = static_cast<float>(p.y());
        pt.z = static_cast<float>(p.z());
        cloud->points.push_back(pt);
    }

    cloud->width  = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

/*//{ sample() */
t_indices RMS::sample(std::vector<Eigen::Vector3d> &frame, std::vector<double> &timestamps) {

  // ROS -> PCL conversion
  t_points pts = vectorEigenToPCL(frame);

  if (pts->empty()) {
    spdlog::warn("[RMS] Empty cloud received. Nothing to sample.");
    return t_indices();
  }

  // Setup a vector of point indices 0->N
  t_indices indices = t_indices(pts->size());
  std::iota(std::begin(indices), std::end(indices), 0);

  // L11: pre-voxelize // frame is already downsampled outside
  // voxelizeIndices(_voxel_input, pts, indices, true);

  // L12-L14: compute GFH
  const auto &gfh = computeGFH(pts, indices);

  // L15-L28: GFH entropy minimization
  indices = sampleByGFH(pts, gfh, size_t(_K), _lambda);

  // post-voxelize if needed
  if (_voxel_output > _voxel_input) {
    voxelizeIndices(_voxel_output, pts, indices, false);
  }

  // L29: sample the input message by indices
  extractByIndices(frame, timestamps, indices);

  return indices;
}
/*//}*/

/*//{ sampleByGFH() */
t_indices RMS::sampleByGFH(const t_points points, const std::vector<t_gfh> &gfh, const size_t K, const float lambda) {

  // Construct histogram of the 1D values
  auto hist = Histogram1D(K, 0.0f, 1.0f, gfh);

  size_t N        = 0;
  float  max_rate = 0.0;

  while (N < points->size()) {

    // Sample one point
    float entropy;
    hist.selectByUniformnessMaximization(entropy);
    N++;

    // Eq. (28): compute mean entropy
    const float rate = entropy / N;

    // L20-L23: Add first K samples to initialize
    if (N <= K) {

      // Eq. (31): Compute maximum entropy rate
      max_rate = std::fmax(rate, max_rate);

    } else {

      // L26: Break if rate of entropy change has slowed down under a threshold
      if ((rate / max_rate) < lambda) {
        break;
      }
    }
  }

  // Retrieve and return the selected indices and return them
  return hist.getSelectedIndices();
}
/*//}*/

/*//{ computeGFH() */
std::vector<t_gfh> RMS::computeGFH(const t_points points_in, const t_indices &indices_in) {

  // Setup output vector
  std::vector<t_gfh> gfh;
  gfh.reserve(indices_in.size());

  std::vector<Eigen::Vector3f> normals;
  std::vector<float> planaritys;
  normals.reserve(indices_in.size());
  planaritys.reserve(indices_in.size());

  // L12: Construct KDTree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // Need to convert our index vector to PCL indices format
  const pcl::IndicesPtr pcl_indices(new pcl::Indices());

  pcl_indices->reserve(indices_in.size());
  for (const auto idx : indices_in) {
    pcl_indices->push_back(idx);
  }

  kdtree.setInputCloud(points_in, pcl_indices);

  float       max_gfh_norm = 0.0f;                // <0, 1> normalizing factor for GFH norm
  const float nn_radius    = 2.0 * _voxel_input;  // Nearest-neighbor search radius
  /* const float nn_radius    = 2.236f * _voxel_input;  // Nearest-neighbor search radius (linear scale: sqrt(5)) */

  for (const auto pt_idx : indices_in) {

    // Get point at index
    const auto &pt = points_in->at(pt_idx);

    // Perform radius search
    std::vector<int>   radius_indices;
    std::vector<float> radius_sq_dist;
    /* kdtree.nearestKSearch(pt, 5, radius_indices, radius_sq_dist); // optional search */
    kdtree.radiusSearch(pt, nn_radius, radius_indices, radius_sq_dist);

    // Compute GFH vector
    size_t          N      = 0;
    Eigen::Vector3f pt_gfh = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < radius_indices.size(); i++) {

      const auto ind = radius_indices[i];

      if (ind == pt_idx) {
        continue;
      }

      const auto &pt_neigh = points_in->at(ind);
      pt_gfh += Eigen::Vector3f(pt_neigh.x - pt.x, pt_neigh.y - pt.y, pt_neigh.z - pt.z);
      N++;
    }

    if (N > 1) {
      pt_gfh /= float(N);
    }

    // Compute the GFH norm
    const float pt_gfh_norm = pt_gfh.norm();
    if (pt_gfh_norm > max_gfh_norm) {
      max_gfh_norm = pt_gfh_norm;
    }

    gfh.emplace_back(pt_idx, pt_gfh_norm);

    // Compute normal and planarity for each point (optional)

    if (N < 6) { 
      normals.emplace_back(Eigen::Vector3f::Zero());
      planaritys.emplace_back(0.0f);
      continue;
    }

    Eigen::Vector3f c = Eigen::Vector3f::Zero();
    // centroid
    for (size_t i = 0; i < radius_indices.size(); i++) {
      const int ind = radius_indices[i];
      if (ind == static_cast<int>(pt_idx)) continue;
      const auto &q = points_in->at(ind);
      c += Eigen::Vector3f(q.x, q.y, q.z);
    }
    c /= float(N);

    // covariance
    Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < radius_indices.size(); i++) {
      const int ind = radius_indices[i];
      if (ind == static_cast<int>(pt_idx)) continue;
      const auto &q = points_in->at(ind);
      Eigen::Vector3f d(q.x - c.x(), q.y - c.y(), q.z - c.z());
      C += d * d.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(C);
    if (es.info() != Eigen::Success) {
      normals.emplace_back(Eigen::Vector3f::Zero());
      planaritys.emplace_back(0.0f);
      continue;
    }

    // eigenvalues are ascending: λ0 ≤ λ1 ≤ λ2
    const auto eval = es.eigenvalues();
    Eigen::Vector3f n = es.eigenvectors().col(0); // smallest eigenvector
    float nrm = n.norm();
    if (!std::isfinite(nrm) || nrm < 1e-6f) {
      normals.emplace_back(Eigen::Vector3f::Zero());
      planaritys.emplace_back(0.0f);
      continue;
    }
    n /= nrm;

    float l0 = eval(0), l1 = eval(1), l2 = eval(2);
    float planarity = (l1 - l0) / (l2 + 1e-12f);  
    planarity = std::max(0.0f, std::min(1.0f, planarity));

    normals.emplace_back(n);
    planaritys.emplace_back(planarity);
  }

  // Normalize to <0, 1>
  if (max_gfh_norm > 0.0f) {
    std::for_each(gfh.begin(), gfh.end(), [&max_gfh_norm](auto &gfh) { gfh.second /= max_gfh_norm; });
  }


  for (size_t i = 0; i < gfh.size(); ++i) {
    const float gfh_norm = gfh[i].second;
    const Eigen::Vector3f n = normals[i];
    const float planarity = planaritys[i];

    if (n.isZero(1e-6f)) continue;

    // Htt += w * (n * n.transpose());
    this->Htt += planarity * (n * n.transpose());
    
  }

  return gfh;
}
/*//}*/

/*//{ extractByIndices() */
void RMS::extractByIndices(std::vector<Eigen::Vector3d> &frame,
                           std::vector<double> &timestamps,
                           const t_indices &indices)
{
  assert(frame.size() == timestamps.size());

  std::vector<Eigen::Vector3d> frame_out;
  std::vector<double>          ts_out;

  frame_out.reserve(indices.size());
  ts_out.reserve(indices.size());

  for (const auto i : indices) {
    assert(i < frame.size());
    frame_out.emplace_back(frame[i]);
    ts_out.emplace_back(timestamps[i]);
  }

  frame      = std::move(frame_out);
  timestamps = std::move(ts_out);
}

/*//}*/

/*//{ voxelizeIndices() */
void RMS::voxelizeIndices(const float voxel_size, const t_points pc_in, t_indices &indices_inout, const bool check_NaNs) {

  // initialize hash set (key: voxel, value: index in pc_in, hashing: VoxelHash)
  tsl::robin_map<t_voxel, size_t, VoxelHash> grid;
  grid.reserve(indices_inout.size());

  // voxelize: keep first point inserted into the voxel
  // other metric for sampling can be used if needed (e.g., the point closest to the voxel center)
  for (const auto idx : indices_inout) {
    const auto &pt = pc_in->at(idx);

    // remove NaNs
    if (check_NaNs && (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))) {
      continue;
    }

    const auto pt_eig = Eigen::Vector3f(pt.x, pt.y, pt.z);
    const auto voxel = t_voxel((pt_eig / voxel_size).cast<int>());

    // voxel occupied: erase this point's index
    if (grid.contains(voxel)) {
      continue;
    }

    // voxel unoccupied: keep this point's index and insert it to the voxel hashset
    grid.insert({voxel, idx});
  }

  // Store voxelized indices
  size_t i = 0;
  for (const auto &key : grid) {
    indices_inout.at(i++) = key.second;
  }
  indices_inout.resize(grid.size());
}
/*//}*/

/*//}*/

}  // namespace rms
