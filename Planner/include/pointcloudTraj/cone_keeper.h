#pragma once

#include <map>
#include <tuple>
#include <Eigen/Eigen>

typedef std::tuple<const Eigen::Vector3d *, int, int> marked_img_pt_type;
typedef std::tuple<const Eigen::Vector3d *, int, int, int, int> marked_img_pt_pair_type;
typedef std::tuple<Eigen::Vector3f, Eigen::Vector3f, double, double> marked_cone_value_type;

class cone_keeper {
public:
    cone_keeper(double init_empty_rad, Eigen::Vector3f init_pos,
                double pixel_cone_angle_2, double focal_distance, int image_width, int image_height);

    void add_marked_img_pts(const Eigen::Affine3f &camera_pose, std::vector<marked_img_pt_type> &&marked_img_pts);

    const std::map<const Eigen::Vector3d *, marked_cone_value_type> &get_marked_cones();

    const float *get_cur_depth_image();

    double get_safe_radius();

    ~cone_keeper();

private:
    void update_marked_cones(const Eigen::Affine3f &camera_pose, const std::vector<marked_img_pt_pair_type> &pts_inter,
                             double &min_p_1, double &min_p_2);

    Eigen::Vector3f to_vec_3d(double x, double y, const Eigen::Matrix3f &rotation);

    double focal_distance;
    int image_width, image_height;

    Eigen::Vector2d l_dir, r_dir;
    Eigen::Matrix2d pixel_norm_l, pixel_norm_r;

    bool was_init;
    double prev_safe_radius, init_empty_rad;
    Eigen::Affine3f prev_camera_pose;
    Eigen::Vector3f init_pos;
    std::vector<marked_img_pt_type> prev_marked_img_pts;

    std::map<const Eigen::Vector3d *, marked_cone_value_type> marked_cones;
    float *cur_depth_image;
};