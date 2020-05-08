#pragma once

#include <list>
#include <vector>
#include <Eigen/Eigen>

typedef std::pair<const Eigen::Affine3f, const float *> image_from_pose_type;
typedef std::list<std::pair<int, int>> points_pixels_type;

class safety_controller {
public:
    safety_controller(int safety_image_width, int safety_image_height, int buffer_size,
                      double focal_distance, int input_image_width, int input_image_height);

    void reset_controlled_points(const std::list<Eigen::Vector3d> &new_points);

    std::tuple<const float *, const points_pixels_type, const points_pixels_type> add_image(
            const Eigen::Affine3f &camera_pose, const float *new_image);

    std::pair<const std::list<Eigen::Vector3d> &, const std::list<Eigen::Vector3d> &> get_groups();

    ~safety_controller();

private:
    bool check_image_for_point(const image_from_pose_type &image, const Eigen::Vector3d &point,
                               int &x, int &y, bool &good) const;

    bool check_image_for_point(const image_from_pose_type &image, const Eigen::Vector3d &point) const;

    int buffer_size;
    int safety_image_width, safety_image_height, input_image_width, input_image_height;
    double focal_distance;

    std::list<image_from_pose_type, Eigen::aligned_allocator<image_from_pose_type>> images_buffer;
    std::list<Eigen::Vector3d> safe_points, unsafe_points;
};
