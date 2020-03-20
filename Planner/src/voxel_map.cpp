#include "pointcloudTraj/voxel_map.h"

voxel_map::voxel_map(double voxel_side) {
    side = voxel_side;
}

void voxel_map::add_point_cloud(const pcl::PointCloud<pcl::PointXYZ> &pcl) {
    for (auto point : pcl) {
        int x = (int) round(point.x / side);
        int y = (int) round(point.y / side);
        int z = (int) round(point.z / side);
        if (map.insert({x, y, z}).second) {
            voxel_cloud.emplace_back(x * side, y * side, z * side);
        }
    }
}

const pcl::PointCloud<pcl::PointXYZ> &voxel_map::get_voxel_cloud() {
    return voxel_cloud;
}