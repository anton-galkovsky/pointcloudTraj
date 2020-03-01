#pragma once

#include <vector>
#include <Eigen/Eigen>

class triangle_shape {
public:
    explicit triangle_shape(const std::vector<Eigen::Vector3f> &sp_points);
    int get_right_border(int y);

    std::vector<Eigen::Vector3f> spatial_points;
    std::vector<Eigen::Vector3f> image_points;
    Eigen::Vector3f normal;
};

class map_observer {
public:
    map_observer(const std::vector<std::vector<Eigen::Vector3f>> &shapes_, int img_width, int img_height, int fov_hor);
    void set_camera_pose(const Eigen::Affine3f &camera_pose);
    float* render_to_image();

private:
    std::vector<triangle_shape> shapes;

    Eigen::Vector3f camera_axis_x;
    Eigen::Vector3f camera_axis_y;
    Eigen::Vector3f camera_axis_z;
    Eigen::Vector3f camera_translation;

    int image_width;
    int image_height;
    float focal_distance;
};