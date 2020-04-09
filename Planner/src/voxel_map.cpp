#include "pointcloudTraj/voxel_map.h"

using namespace std;

inline void to_voxel_components(int &x, int &y, int &z, const pcl::PointXYZ &point, double res) {
    x = (int) round(point.x / res);
    y = (int) round(point.y / res);
    z = (int) round(point.z / res);
}

template<class T>
inline void to_voxel_components(int &x, int &y, int &z, const T &point, double res) {
    x = (int) round(point[0] / res);
    y = (int) round(point[1] / res);
    z = (int) round(point[2] / res);
}

template<class Cont2>
voxel_map<Cont2>::voxel_map(double res) :
        res(res) {
}

template<class Cont2>
template<class Cont1>
void voxel_map<Cont2>::add_point_cloud(const Cont1 &pcl) {
    int x = 0, y = 0, z = 0;
    for (const auto &point : pcl) {
        to_voxel_components(x, y, z, point, res);
        if (map.insert({x, y, z}).second) {
            voxel_cloud.emplace_back(x * res, y * res, z * res);
        }
    }
}

template<class Cont2>
template<class T>
bool voxel_map<Cont2>::add_point(const T &point) {
    int x = 0, y = 0, z = 0;
    to_voxel_components(x, y, z, point, res);
    if (map.insert({x, y, z}).second) {
        voxel_cloud.emplace_back(x * res, y * res, z * res);
        return true;
    }
    return false;
}

template<class Cont2>
const Cont2 &voxel_map<Cont2>::get_voxel_cloud() {
    return voxel_cloud;
}

template<class Cont2>
template<class Cont1>
Cont2 voxel_map<Cont2>::to_voxel_cloud(const Cont1 &pcl, double res) {
    voxel_map<Cont2> map(res);
    map.add_point_cloud(pcl);
    return map.get_voxel_cloud();
}

typedef pcl::PointCloud<pcl::PointXYZ> pcl_p;
typedef vector<Eigen::Vector3d>        vec_d;
typedef vector<Eigen::Vector3f>        vec_f;

template void voxel_map<pcl_p>::add_point_cloud<pcl_p>(const pcl_p &pcl);
template void voxel_map<pcl_p>::add_point_cloud<vec_d>(const vec_d &pcl);
template bool voxel_map<pcl_p>::add_point<Eigen::Vector3f>(const Eigen::Vector3f &point);
template bool voxel_map<pcl_p>::add_point<Eigen::Vector3d>(const Eigen::Vector3d &point);
template vec_d voxel_map<vec_d>::to_voxel_cloud<vec_d>(const vec_d &pcl, double res);
template vec_f voxel_map<vec_f>::to_voxel_cloud<vec_f>(const vec_f &pcl, double res);
template void to_voxel_components<Eigen::Vector3d>(int &x, int &y, int &z, const Eigen::Vector3d &point, double res);
template void to_voxel_components<Eigen::Vector3f>(int &x, int &y, int &z, const Eigen::Vector3f &point, double res);

template class voxel_map<pcl_p>;
template class voxel_map<vec_d>;
template class voxel_map<vec_f>;