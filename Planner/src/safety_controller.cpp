#include "pointcloudTraj/safety_controller.h"

using namespace std;

safety_controller::safety_controller(int safety_image_width, int safety_image_height, int buffer_size,
                                     double focal_distance, int input_image_width, int input_image_height) :
        buffer_size(buffer_size), safety_image_width(safety_image_width), safety_image_height(safety_image_height),
        input_image_width(input_image_width), input_image_height(input_image_height), focal_distance(focal_distance) {
}

void safety_controller::reset_controlled_points(const std::list<Eigen::Vector3d> &new_points) {
    safe_points.clear();
    unsafe_points = new_points;
    for (auto it = unsafe_points.begin(); it != unsafe_points.end();) {
        bool safe = false;
        for (const auto &image : images_buffer) {
            safe = check_image_for_point(image, *it);
            if (safe) {
                break;
            }
        }
        if (safe) {
            safe_points.push_back(*it);
            it = unsafe_points.erase(it);
        } else {
            ++it;
        }
    }
}

tuple<const float *, const points_pixels_type, const points_pixels_type> safety_controller::add_image(
        const Eigen::Affine3f &camera_pose, const float *new_image) {
    auto new_safety_image = new float[safety_image_width * safety_image_height];
    for (int i = 0; i < safety_image_width * safety_image_height; i++) {
        new_safety_image[i] = INFINITY;
    }
    double width_scale = 1.0 * safety_image_width / input_image_width;
    double height_scale = 1.0 * safety_image_height / input_image_height;
    for (int i = 0; i < input_image_width; i++) {
        for (int j = 0; j < input_image_height; j++) {
            int input_idx = j * input_image_width + i;
            int safety_idx = (int) (j * height_scale) * safety_image_width + (int) (i * width_scale);
            if (new_image[input_idx] < new_safety_image[safety_idx]) {
                new_safety_image[safety_idx] = new_image[input_idx];
            }
        }
    }

    image_from_pose_type new_image_from_pose({camera_pose, new_safety_image});

    auto new_image_info = tuple<const float *, points_pixels_type, points_pixels_type>();
    get<0>(new_image_info) = new_safety_image;
    get<1>(new_image_info).clear();
    get<2>(new_image_info).clear();

    int x, y;
    bool good;
    for (auto it = unsafe_points.begin(); it != unsafe_points.end();) {
        bool safe = check_image_for_point(new_image_from_pose, *it, x, y, good);
        if (safe) {
            if (good) {
                get<1>(new_image_info).push_back({x, y});
            }
            safe_points.push_back(*it);
            it = unsafe_points.erase(it);
        } else {
            if (good) {
                get<2>(new_image_info).push_back({x, y});
            }
            ++it;
        }
    }
    for (auto it = safe_points.begin(); it != safe_points.end(); ++it) {
        check_image_for_point(new_image_from_pose, *it, x, y, good);
        if (good) {
            get<1>(new_image_info).push_back({x, y});
        }
    }

    images_buffer.push_back(new_image_from_pose);
    if (buffer_size < (int)images_buffer.size()) {
        image_from_pose_type first_img = images_buffer.front();
        delete[] first_img.second;
        images_buffer.pop_front();
    }

    return new_image_info;
}

std::pair<const std::list<Eigen::Vector3d> &, const std::list<Eigen::Vector3d> &> safety_controller::get_groups() {
    return {safe_points, unsafe_points};
}

safety_controller::~safety_controller() {
    while (!images_buffer.empty()) {
        image_from_pose_type first_img = images_buffer.front();
        delete[] first_img.second;
        images_buffer.pop_front();
    }
}

bool safety_controller::check_image_for_point(const image_from_pose_type &image, const Eigen::Vector3d &point,
                                              int &x, int &y, bool &good) const {
    x = y = -1;
    good = false;

    Eigen::Vector3f delta = point.cast<float>() - image.first.translation();
    const auto &rotation = image.first.rotation();
    double distance_z = delta.dot(Eigen::Vector3f(rotation.col(2)));
    if (distance_z < 0.01) {
        return false;
    }
    double scale = focal_distance / distance_z * safety_image_width;
    int img_x = (int) round(delta.dot(Eigen::Vector3f(rotation.col(0))) * scale + safety_image_width / 2.0);
    if (img_x < 0 || img_x >= safety_image_width) {
        return false;
    }
    int img_y = (int) round(delta.dot(Eigen::Vector3f(rotation.col(1))) * scale + safety_image_height / 2.0);
    if (img_y < 0 || img_y >= safety_image_height) {
        return false;
    }

    x = img_x;
    y = img_y;
    good = true;
    if (image.second[img_y * safety_image_width + img_x] == INFINITY) {
        return false;
    }
    return delta.norm() < image.second[img_y * safety_image_width + img_x];
}

bool safety_controller::check_image_for_point(const image_from_pose_type &image, const Eigen::Vector3d &point) const {
    int x, y;
    bool good;
    return check_image_for_point(image, point, x, y, good);
}