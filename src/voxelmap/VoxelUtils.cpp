#include "voxelmap/VoxelUtils.hpp"

#include <tsl/robin_map.h>

namespace ns_river {

// std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
//                                              const double voxel_size) {
//     tsl::robin_map<Voxel, Eigen::Vector3d> grid;
//     grid.reserve(frame.size());
//     std::for_each(frame.cbegin(), frame.cend(), [&](const auto &point) {
//         const auto voxel = PointToVoxel(point, voxel_size);
//         if (!grid.contains(voxel)) grid.insert({voxel, point});
//     });
//     std::vector<Eigen::Vector3d> frame_dowsampled;
//     frame_dowsampled.reserve(grid.size());
//     std::for_each(grid.cbegin(), grid.cend(), [&](const auto &voxel_and_point) {
//         frame_dowsampled.emplace_back(voxel_and_point.second);
//     });
//     return frame_dowsampled;
// }
std::pair<std::vector<Eigen::Vector3d>, std::vector<double>>
VoxelDownsample(const std::vector<Eigen::Vector3d> &frame, const std::vector<double> &timestamps,
                                             const double voxel_size) {
    tsl::robin_map<Voxel, std::pair<Eigen::Vector3d, std::size_t>> grid;
    grid.reserve(frame.size());
    for (std::size_t i = 0; i < frame.size(); ++i) {
    const auto v = PointToVoxel(frame[i], voxel_size);
        if (!grid.contains(v)) grid.insert({v, {frame[i], i}});
    }
    std::vector<Eigen::Vector3d> frame_dowsampled;
    std::vector<double>          ts_dowsampled;
    frame_dowsampled.reserve(grid.size());
    std::for_each(grid.cbegin(), grid.cend(), [&](const auto &voxel_and_point) {
        frame_dowsampled.emplace_back(voxel_and_point.second.first);
        ts_dowsampled.emplace_back(timestamps[voxel_and_point.second.second]);

    });
    return {std::move(frame_dowsampled), std::move(ts_dowsampled)};
}

} 
