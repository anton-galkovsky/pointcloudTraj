#include "pointcloudTraj/cone_keeper.h"
#include "pointcloudTraj/utils.h"

using namespace std;

cone_keeper::cone_keeper(double init_empty_rad, Eigen::Vector3f init_pos,
                         double pixel_cone_angle_2, double focal_distance, int image_width, int image_height) :
        focal_distance(focal_distance), image_width(image_width), image_height(image_height), was_init(false),
        prev_safe_radius(init_empty_rad), init_empty_rad(init_empty_rad), init_pos(init_pos) {
    cur_depth_image = new float[image_width * image_height];

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

void cone_keeper::add_marked_img_pts(const Eigen::Affine3f &camera_pose,
                                     vector<marked_img_pt_type> &&marked_img_pts) {
    for (int i = 0; i < image_width * image_height; i++) {
        cur_depth_image[i] = INFINITY;
    }
    sort(marked_img_pts.begin(), marked_img_pts.end());

    if (was_init) {
        double translation_delta = (camera_pose.translation() - prev_camera_pose.translation()).norm();
        double rotation_delta = camera_pose.rotation().col(2).cross(prev_camera_pose.rotation().col(2)).norm();
        if (translation_delta > 0.005 || rotation_delta > 0.01) {
            auto pts_inter = intersect_sorted_marked_img_pts(prev_marked_img_pts, marked_img_pts);
            double min_p_2, min_p_1;
            min_p_2 = min_p_1 = 1000000;
            update_marked_cones(camera_pose, pts_inter, min_p_1, min_p_2);

            double safe_radius = 1000000;
            for (auto it : marked_cones) {
                double h_1 = get<2>(it.second);
                Eigen::Vector3f c = get<0>(it.second);
                Eigen::Vector3f v = get<1>(it.second);
                Eigen::Vector3f v1 = c + v * h_1;
                safe_radius = min(safe_radius, (double) (v1 - camera_pose.translation()).norm());
            }
            if (min_p_1 > 999999) {
                min_p_1 = safe_radius;
            }
            prev_safe_radius = min(max(max(safe_radius, min_p_1), 0.01), 99.0);
        }
    }
    was_init = true;
    prev_camera_pose = camera_pose;
    prev_marked_img_pts = marked_img_pts;
}

const map<const Eigen::Vector3d *, marked_cone_value_type> &cone_keeper::get_marked_cones() {
    return marked_cones;
}

const float *cone_keeper::get_cur_depth_image() {
    return cur_depth_image;
}

double cone_keeper::get_safe_radius() {
    return prev_safe_radius;
}

cone_keeper::~cone_keeper() {
    delete[] cur_depth_image;
}

void cone_keeper::update_marked_cones(const Eigen::Affine3f &camera_pose,
                                      const vector<marked_img_pt_pair_type> &pts_inter,
                                      double &min_p_1, double &min_p_2) {
    double dist_from_init = (camera_pose.translation() - init_pos).norm();
    for (const auto &pt : pts_inter) {
        auto marked_cone_it = marked_cones.find(get<0>(pt));
        marked_cone_value_type *cone_0;
        if (marked_cone_it == marked_cones.end()) {
            double init_dist;
            if (abs((double) get<1>(pt) / image_width - 0.5) < 0.4
                && abs((double) get<2>(pt) / image_height - 0.5) < 0.4) {
                init_dist = prev_safe_radius;
            } else {
                init_dist = 0.01;
            }
            if (dist_from_init < init_empty_rad) {
                init_dist = max(init_dist, init_empty_rad - dist_from_init);
            }
            Eigen::Vector3f a = to_vec_3d(get<1>(pt), get<2>(pt), prev_camera_pose.rotation()).normalized();
            cone_0 = &marked_cones.insert({get<0>(pt),
                                           {prev_camera_pose.translation(), a, init_dist, 1000}}).first->second;
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
        points.emplace_back(l_dir * h_1);
        points.emplace_back(r_dir * h_1);
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

        cur_depth_image[get<2>(pt) * image_width + get<1>(pt)] = min_p;

        if (max_c - min_c <= h_2 - h_1) {
            get<0>(*cone_0) = camera_pose.translation();
            get<1>(*cone_0) = v;
            get<2>(*cone_0) = min_p;
            get<3>(*cone_0) = max_p;

            if (min_p_2 > max_p) {
                min_p_2 = max_p;
                min_p_1 = min_p;
            }
        } else {
            get<2>(*cone_0) = min_c;
            get<3>(*cone_0) = max_c;
        }
    }
}