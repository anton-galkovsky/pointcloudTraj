#pragma once

#include <map>
#include <tuple>
#include <Eigen/Eigen>

typedef std::tuple<const Eigen::Vector3d *, int, int> marked_img_pt_type;
typedef std::tuple<const Eigen::Vector3d *, int, int, int, int> marked_img_pt_pair_type;
typedef std::tuple<Eigen::Vector3f, Eigen::Vector3f, double, double> marked_cone_value_type;

class base_keeper {
public:
    base_keeper(Eigen::Vector3f init_pos,
                double pixel_cone_angle_2, double focal_distance, int image_width, int image_height);

    const std::map<const Eigen::Vector3d *, marked_cone_value_type> &get_marked_cones();

    virtual void add_marked_img_pts(const Eigen::Affine3f &camera_pose,
                                    std::vector<marked_img_pt_type> &&marked_img_pts) = 0;

    virtual const float *get_cur_depth_image() = 0;

    virtual double get_safe_radius() = 0;

    virtual ~base_keeper();

protected:
    Eigen::Vector3f to_vec_3d(double x, double y, const Eigen::Matrix3f &rotation);

    double focal_distance;
    int image_width, image_height;

    Eigen::Vector2d l_dir, r_dir;
    Eigen::Matrix2d pixel_norm_l, pixel_norm_r;

    bool was_init;
    Eigen::Vector3f init_pos;

    std::map<const Eigen::Vector3d *, marked_cone_value_type> marked_cones;
};

class cone_keeper : public base_keeper {
public:
    cone_keeper(double init_empty_rad, Eigen::Vector3f init_pos,
                double pixel_cone_angle_2, double focal_distance, int image_width, int image_height);

    void add_marked_img_pts(const Eigen::Affine3f &camera_pose,
                            std::vector<marked_img_pt_type> &&marked_img_pts) override;

    const float *get_cur_depth_image() override;

    double get_safe_radius() override;

    ~cone_keeper();

private:
    void update_marked_cones(const Eigen::Affine3f &camera_pose, const std::vector<marked_img_pt_pair_type> &pts_inter,
                             double &min_p_1, double &min_p_2);

    double prev_safe_radius, init_empty_rad;
    Eigen::Affine3f prev_camera_pose;
    std::vector<marked_img_pt_type> prev_marked_img_pts;
    float *cur_depth_image;
};


class pixel_keeper : public base_keeper {
public:
    pixel_keeper(int window, int min_disp, Eigen::Vector3f init_pos,
                 double pixel_cone_angle_2, double focal_distance, int image_width, int image_height);

    void add_marked_img_pts(const Eigen::Affine3f &camera_pose,
                            std::vector<marked_img_pt_type> &&marked_img_pts) override;

    const float *get_cur_depth_image() override;

    double get_safe_radius() override;

    ~pixel_keeper();

private:
    int window, cur_num, min_disp;
    std::map<const Eigen::Vector3d *, std::list<std::tuple<int, int, int>>> pixel_bufs;
    std::list<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> pose_buf;
};