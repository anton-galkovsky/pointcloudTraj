#pragma once

#include <vector>
#include <set>
#include <array>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<class Cont2>
class voxel_map {
public:
    explicit voxel_map(double res);

    template<class Cont1>
    void add_point_cloud(const Cont1 &pcl);

    const Cont2 &get_voxel_cloud();

    template<class Cont1>
    static Cont2 to_voxel_cloud(const Cont1 &pcl, double res);

private:
    std::set<std::array<int, 3>> map;
    Cont2 voxel_cloud;
    double res;
};

typedef voxel_map<pcl::PointCloud<pcl::PointXYZ>> voxel_map_pcl;
typedef voxel_map<std::vector<Eigen::Vector3d>> voxel_map_vec_d;
typedef voxel_map<std::vector<Eigen::Vector3f>> voxel_map_vec_f;