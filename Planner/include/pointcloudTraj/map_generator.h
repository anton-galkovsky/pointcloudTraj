#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class map_generator {
public:
    map_generator(double x_init, double x_end, double y_init, double y_end,
                  double x1, double x2, double y1, double y2, double h1, double h2,
                  double w1, double w2, double res, int num, int seed = -1);

    void generate_map();

    const std::vector<std::vector<Eigen::Vector3d>> *get_shapes();

    const pcl::PointCloud<pcl::PointXYZ> *get_global_map_pcl();

    ~map_generator();

private:
    void emplace_rect_to_map(double x_1, double y_1, double x_2, double y_2, double h_);

    double x_init, x_end;
    double y_init, y_end;
    double x1, x2;
    double y1, y2;
    double h1, h2;
    double w1, w2;
    double res;
    int num;
    int seed;

    std::vector<std::vector<Eigen::Vector3d>> *shapes;
    pcl::PointCloud<pcl::PointXYZ> *global_map_pcl;
};