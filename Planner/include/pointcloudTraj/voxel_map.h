#pragma once

#include <vector>
#include <set>
#include <array>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class voxel_map {
public:
    explicit voxel_map(double voxel_side);

    void add_point_cloud(const pcl::PointCloud<pcl::PointXYZ> &pcl);

    const pcl::PointCloud<pcl::PointXYZ> &get_voxel_cloud();

private:
    std::set<std::array<int, 3>> map;
    pcl::PointCloud<pcl::PointXYZ> voxel_cloud;
    double side;
};