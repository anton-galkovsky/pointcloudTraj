#pragma once

#include <vector>
#include <set>
#include <map>
#include <array>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<class Cont2>
class voxel_map {
public:
    explicit voxel_map(double res);

    template<class Cont1>
    void add_point_cloud(const Cont1 &pcl);

    template<class T>
    bool add_point(const T &point);

    const Cont2 &get_voxel_cloud();

    template<class Cont1>
    static Cont2 to_voxel_cloud(const Cont1 &pcl, double res);

private:
    std::set<std::array<int, 3>> map;
    Cont2 voxel_cloud;
    double res;
};

class voxel_value_map {
public:
    explicit voxel_value_map(double res);

    int add_point(const Eigen::Vector3d &point);

    const pcl::PointCloud<pcl::PointXYZ> &get_voxel_cloud();

private:
    std::map<std::array<int, 3>, int> map;
    pcl::PointCloud<pcl::PointXYZ> voxel_cloud;
    double res;
};

typedef voxel_map<pcl::PointCloud<pcl::PointXYZ>> voxel_map_pcl;
typedef voxel_map<std::vector<Eigen::Vector3d>> voxel_map_vec_d;
typedef voxel_map<std::vector<Eigen::Vector3f>> voxel_map_vec_f;