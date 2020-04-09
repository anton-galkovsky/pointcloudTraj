#include "pointcloudTraj/cone_keeper.h"
#include "pointcloudTraj/utils.h"

using namespace std;

cone_keeper::cone_keeper(double pixel_cone_angle_2, double focal_distance, int image_width, int image_height) :
        focal_distance(focal_distance), image_width(image_width), image_height(image_height), was_init(false) {
    l_dir << -pixel_cone_angle_2, 1;
    r_dir << +pixel_cone_angle_2, 1;
    double cos_angle = cos(pixel_cone_angle_2);
    double sin_angle = sin(pixel_cone_angle_2);
    pixel_norm_l << sin_angle, +cos_angle, -cos_angle, sin_angle;
    pixel_norm_r << sin_angle, -cos_angle, +cos_angle, sin_angle;
}

Eigen::Vector3f cone_keeper::to_vec_3d(double x, double y, const Eigen::Matrix3f &rotation) {
    return (x / image_width - 0.5) / focal_distance * rotation.col(0) +
           (y - 0.5 * image_height) / image_width / focal_distance * rotation.col(1) +
           rotation.col(2);
}

void cone_keeper::add_marked_img_pts(const Eigen::Affine3f &camera_pose, vector<marked_img_pt_type> &&marked_img_pts) {
    sort(marked_img_pts.begin(), marked_img_pts.end());

    if (was_init) {
        double translation_delta = (camera_pose.translation() - prev_camera_pose.translation()).norm();
        double rotation_delta = camera_pose.rotation().col(2).cross(prev_camera_pose.rotation().col(2)).norm();
        if (translation_delta > 0.005 || rotation_delta > 0.01) {
            auto pts_inter = intersect_sorted_marked_img_pts(prev_marked_img_pts, marked_img_pts);
            update_marked_cones(camera_pose, pts_inter);
        }
    }
    was_init = true;
    prev_camera_pose = camera_pose;
    prev_marked_img_pts = marked_img_pts;
}

const map<const Eigen::Vector3d *, marked_cone_value_type> &cone_keeper::get_marked_cones() {
    return marked_cones;
}

void cone_keeper::update_marked_cones(const Eigen::Affine3f &camera_pose,
                                      const vector<marked_img_pt_pair_type> &pts_inter) {
    for (const auto &pt : pts_inter) {
        auto marked_cone_it = marked_cones.find(get<0>(pt));
        marked_cone_value_type *cone_0;
        if (marked_cone_it == marked_cones.end()) {
            Eigen::Vector3f a = to_vec_3d(get<1>(pt), get<2>(pt), prev_camera_pose.rotation()).normalized();
            cone_0 = &marked_cones.insert({get<0>(pt),
                                           {prev_camera_pose.translation(), a, 0.01, 100}}).first->second;
        } else {
            cone_0 = &marked_cone_it->second;
        }

        Eigen::Vector3f _c = get<0>(*cone_0);
        Eigen::Vector3f _p = camera_pose.translation();
        Eigen::Vector3f a = get<1>(*cone_0);
        Eigen::Vector3f v = to_vec_3d(get<3>(pt), get<4>(pt), camera_pose.rotation()).normalized();
        Eigen::Vector3f r = a.cross(v).normalized();
        Eigen::Vector3f b = a.cross(r);
        Eigen::Matrix<float, 2, 3> camera_basis_2d;
        camera_basis_2d << b[0], b[1], b[2], a[0], a[1], a[2];
        Eigen::Vector2d _p_ = (camera_basis_2d * (_p - _c)).cast<double>();
        Eigen::Vector2d v_ = (camera_basis_2d * v * 100).cast<double>();

        double h_1 = get<2>(*cone_0);
        double h_2 = get<3>(*cone_0);
        list<Eigen::Vector2d> points;
        points.emplace_back(0, 0);
        points.emplace_back(r_dir * h_2);
        points.emplace_back(l_dir * h_2);

        Eigen::Vector2d p_n_l = pixel_norm_l * v_;
        Eigen::Vector2d p_n_r = pixel_norm_r * v_;
        intersect_points_with_plane(points, _p_, p_n_l);
        intersect_points_with_plane(points, _p_, p_n_r);

        v_.normalize();
        double min_c, max_c, min_p, max_p;
        min_c = max_c = (*points.begin())[1];
        min_p = max_p = (*points.begin() - _p_).dot(v_);
        for (auto it = ++points.begin(); it != points.end(); ++it) {
            double d_c = (*it)[1];
            double d_p = (*it - _p_).dot(v_);
            min_c = min(min_c, d_c);
            min_p = min(min_p, d_p);
            max_c = max(max_c, d_c);
            max_p = max(max_p, d_p);
        }
        min_c = max(min_c, h_1);
        double volume_0 = h_2 * h_2 * (h_2 - h_1);
        double volume_c = max_c * max_c * (max_c - min_c);
        double volume_p = max_p * max_p * (max_p - min_p);
        if (volume_p <= volume_c && volume_p <= volume_0) {
            get<0>(*cone_0) = camera_pose.translation();
            get<1>(*cone_0) = v;
            get<2>(*cone_0) = min_p;
            get<3>(*cone_0) = max_p;
        } else if (volume_c <= volume_0) {
            get<2>(*cone_0) = min_c;
            get<3>(*cone_0) = max_c;
        }
    }
}