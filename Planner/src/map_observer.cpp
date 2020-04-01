#include <ros/ros.h>
#include <list>
#include <algorithm>
//#include <iostream>
#include "pointcloudTraj/map_observer.h"

using namespace std;

img_map_observer::img_map_observer(const vector<vector<Eigen::Vector3d>> &shapes_,
                                   int img_width, int img_height, int fov_hor) :
        map_observer(shapes_, img_width, img_height, fov_hor) {
    image = new float[image_width * image_height];
}

const float *img_map_observer::render_to_img() {
    for (int i = 0; i < image_width * image_height; i++) {
        image[i] = INFINITY;
    }
    render();
    return image;
}

img_map_observer::~img_map_observer() {
    delete[] image;
}

void img_map_observer::save_point(int x, int y, double d) {
    image[y * image_width + x] = (float) d;
}


pcl_map_observer::pcl_map_observer(const vector<vector<Eigen::Vector3d>> &shapes_,
                                   int img_width, int img_height, int fov_hor) :
        map_observer(shapes_, img_width, img_height, fov_hor) {
    pcl.height = 1;
    pcl.is_dense = true;
}

const pcl::PointCloud<pcl::PointXYZ> *pcl_map_observer::render_to_pcl() {
    pcl.points.clear();
    render();
    pcl.width = pcl.size();
    return &pcl;
}

void pcl_map_observer::save_point(int x, int y, double d) {
    Eigen::Vector3d p = camera_translation + d *
                                             (((double) x / image_width - 0.5) / focal_distance * camera_axis_x +
                                              (y - 0.5 * image_height) / image_width / focal_distance * camera_axis_y +
                                              camera_axis_z);
    pcl.points.emplace_back(p[0], p[1], p[2]);
}


img_pcl_map_observer::img_pcl_map_observer(const vector<vector<Eigen::Vector3d>> &shapes_,
                                           int img_width, int img_height, int fov_hor) :
        map_observer(shapes_, img_width, img_height, fov_hor) {
    image = new float[image_width * image_height];
    pcl.height = 1;
    pcl.is_dense = true;
    actual_rendering_data = false;
}

const float *img_pcl_map_observer::render_to_img() {
    if (!actual_rendering_data) {
        actual_rendering_data = true;
        for (int i = 0; i < image_width * image_height; i++) {
            image[i] = INFINITY;
        }
        pcl.points.clear();
        render();
        pcl.width = pcl.size();
    }
    return image;
}

const pcl::PointCloud<pcl::PointXYZ> *img_pcl_map_observer::render_to_pcl() {
    render_to_img();
    return &pcl;
}

img_pcl_map_observer::~img_pcl_map_observer() {
    delete[] image;
}

void img_pcl_map_observer::save_point(int x, int y, double d) {
    image[y * image_width + x] = (float) d;

    Eigen::Vector3d p = camera_translation + d *
                                             (((double) x / image_width - 0.5) / focal_distance * camera_axis_x +
                                              (y - 0.5 * image_height) / image_width / focal_distance * camera_axis_y +
                                              camera_axis_z);
    pcl.points.emplace_back(p[0], p[1], p[2]);
}

void img_pcl_map_observer::set_camera_pose(const Eigen::Affine3f &camera_pose) {
    map_observer::set_camera_pose(camera_pose);
    actual_rendering_data = false;
}


marked_map_observer::marked_map_observer(const vector<vector<Eigen::Vector3d>> &marked_points,
                                         const vector<vector<Eigen::Vector3d>> &shapes_,
                                         int img_width, int img_height, int fov_hor) :
        map_observer(shapes_, img_width, img_height, fov_hor), marked_points(marked_points), marked_img_pts(nullptr) {
    marks_image = new pair<const Eigen::Vector3d *, float>[img_width * img_height];
}

void marked_map_observer::render_to_marked_img_pts(vector<tuple<const Eigen::Vector3d *, int, int>> &marked_img_pts_) {
    for (int i = 0; i < image_width * image_height; i++) {
        marks_image[i] = {nullptr, INFINITY};
    }
    marked_img_pts = &marked_img_pts_;
    marked_img_pts->clear();
    render();
}

marked_map_observer::~marked_map_observer() {
    delete[] marks_image;
}

void marked_map_observer::save_point(int x, int y, double d) {
    auto ptr = marks_image[y * image_width + x].first;
    float dist = marks_image[y * image_width + x].second;
    if (ptr != nullptr && abs(dist - d) < 0.05) {
        marked_img_pts->emplace_back(ptr, x, y);
    }
}

void marked_map_observer::operate_marked(int shape_idx) {
    for (const auto &point : marked_points[shape_idx]) {
        Eigen::Vector3d delta = point - camera_translation;
        if (delta.dot(cone_normals[0]) < -0.01 ||
            delta.dot(cone_normals[1]) < -0.01 ||
            delta.dot(cone_normals[2]) < -0.01 ||
            delta.dot(cone_normals[3]) < -0.01) {
            continue;
        }
        double distance_z = max(delta.dot(camera_axis_z), 0.01);
        double scale = focal_distance / distance_z * image_width;
        int img_x = (int) round(delta.dot(camera_axis_x) * scale + image_width / 2.0);
        if (img_x >= image_width || img_x < 0) {
            continue;
        }
        int img_y = (int) round(delta.dot(camera_axis_y) * scale + image_height / 2.0);
        if (img_y >= image_height || img_y < 0) {
            continue;
        }
        if (distance_z < marks_image[img_y * image_width + img_x].second) {
            marks_image[img_y * image_width + img_x] = {&point, distance_z};
        }
    }
}

double marked_map_observer::get_focal_distance() {
    return focal_distance;
}


map_observer::plane_convex_shape::plane_convex_shape(const vector<Eigen::Vector3d> &sp_points) {
    spatial_points = sp_points;
    normal = (spatial_points[1] - spatial_points[0]).cross(spatial_points[2] - spatial_points[1]);
}


inline double intermediate_distance(int i, int n, double v0_dist, double v1_dist) {
    if (n == 0) {
        return v0_dist;
    }
    return v0_dist * v1_dist / (v1_dist + (v0_dist - v1_dist) * i / n);
}

inline double intermediate_distance(int x_cur, const segment_type *segment) {
    return intermediate_distance(x_cur - get<0>(*segment),
                                 get<2>(*segment) - get<0>(*segment),
                                 get<1>(*segment),
                                 get<3>(*segment));
}

void map_observer::operate_marked(int) {
}

inline double intermediate_distance(double alpha, double v0_dist, double v1_dist) {
    return v0_dist * v1_dist / (v1_dist + (v0_dist - v1_dist) * alpha);
}

void map_observer::intersect_points(list<Eigen::Vector3d> &points,
                                    const Eigen::Vector3d &plane_point, const Eigen::Vector3d &normal) {
    for (auto it_1 = points.begin(), it = it_1; it != points.end();) {
        ++it_1;
        if (it_1 == points.end()) {
            it_1 = points.begin();
        }
        double n_dot_i = (*it - plane_point).dot(normal);
        double n_dot_i_1 = (*it_1 - plane_point).dot(normal);
        if ((n_dot_i < -0.01 && n_dot_i_1 > 0.01) || (n_dot_i_1 < -0.01 && n_dot_i > 0.01)) {
            auto new_it = points.insert(it_1, *it - (*it_1 - *it) * n_dot_i / (*it_1 - *it).dot(normal));
            if (new_it == points.begin()) {
                break;
            } else {
                it = it_1;
            }
        } else {
            ++it;
        }
    }
    for (auto it = points.begin(); it != points.end();) {
        if ((*it - plane_point).dot(normal) < -0.01) {
            it = points.erase(it);
        } else {
            ++it;
        }
    }
}

vector<Eigen::Vector3d> map_observer::get_points_in_cone(const vector<Eigen::Vector3d> &points) {
    list<Eigen::Vector3d> cone_points;
    for (const auto &point : points) {
        cone_points.push_back(point);
    }

    for (const auto &cone_normal : cone_normals) {
        intersect_points(cone_points, camera_translation, cone_normal);
    }

    vector<Eigen::Vector3d> ans_points;
    for (const auto &point : cone_points) {
        ans_points.push_back(point);
    }
    return ans_points;
}

void map_observer::recount_cone_params() {
    cone_normals[0] = camera_axis_z * tan(1.0 * hor_angle_2) + camera_axis_x;
    cone_normals[1] = camera_axis_z * tan(1.0 * hor_angle_2) - camera_axis_x;
    cone_normals[2] = camera_axis_z * tan(1.0 * ver_angle_2) + camera_axis_y;
    cone_normals[3] = camera_axis_z * tan(1.0 * ver_angle_2) - camera_axis_y;
}

map_observer::map_observer(const vector<vector<Eigen::Vector3d>> &shapes_, int img_width, int img_height, int fov_hor) :
        image_width(img_width), image_height(img_height) {
    hor_angle_2 = fov_hor / 2.0 * M_PI / 180;
    focal_distance = 0.5 / tan(1.0 * hor_angle_2);
    ver_angle_2 = atan2(0.5 * image_height / image_width, focal_distance);
    camera_translation = Eigen::Vector3d(0, 0, 0);
    camera_axis_x = Eigen::Vector3d(1, 0, 0);
    camera_axis_y = Eigen::Vector3d(0, 1, 0);
    camera_axis_z = Eigen::Vector3d(0, 0, 1);

    recount_cone_params();

    for (const auto &shape : shapes_) {
        shapes.emplace_back(shape);
    }
}

void map_observer::set_camera_pose(const Eigen::Affine3f &camera_pose) {
    camera_translation = camera_pose.translation().cast<double>();
    camera_axis_x = Eigen::Vector3d(camera_pose.rotation().cast<double>() * Eigen::Vector3d(1, 0, 0));
    camera_axis_y = Eigen::Vector3d(camera_pose.rotation().cast<double>() * Eigen::Vector3d(0, 1, 0));
    camera_axis_z = Eigen::Vector3d(camera_pose.rotation().cast<double>() * Eigen::Vector3d(0, 0, 1));

    recount_cone_params();
}

void map_observer::render() {
    auto *line_segments = new map<plane_convex_shape *, segment_type>[image_height];

//    double t[7]{};
//    ros::Time t0 = ros::Time::now();
    int shape_idx = -1;
    for (auto &shape : shapes) {
        shape_idx++;
        if ((shape.spatial_points[0] - camera_translation).dot(shape.normal) < 0) {
            vector<Eigen::Vector3d> cone_points = get_points_in_cone(shape.spatial_points);

            shape.image_points.clear();
            for (const auto &point : cone_points) {
                Eigen::Vector3d delta = point - camera_translation;
                double distance_z = max(delta.dot(camera_axis_z), 0.01);
                double scale = focal_distance / distance_z * image_width;
                double img_x = delta.dot(camera_axis_x) * scale + image_width / 2.0;
                double img_y = delta.dot(camera_axis_y) * scale + image_height / 2.0;
                shape.image_points.emplace_back(img_x, img_y, distance_z);
            }

            operate_marked(shape_idx);

            int image_points_size = shape.image_points.size();
            for (int i = 0; i < image_points_size; i++) {
                double x0 = shape.image_points[i][0];
                double y0 = shape.image_points[i][1];
                double z0 = shape.image_points[i][2];
                double x1 = shape.image_points[(i + 1) % image_points_size][0];
                double y1 = shape.image_points[(i + 1) % image_points_size][1];
                double z1 = shape.image_points[(i + 1) % image_points_size][2];
                double x_incr, y_incr;
                double x = x0;
                double y = y0;
                double z = shape.image_points[i][2];
                double alpha = 0;
                double alpha_incr;
                bool step_on_x;
                if (abs(x1 - x0) < abs(y1 - y0)) {
                    double dy = round(y) - y;
                    y = round(y);
                    y_incr = (y1 > y) ? 1 : -1;
                    if (abs(y1 - y) < 0.1) {
                        x_incr = x1 - x0;
                        alpha_incr = 1;
                    } else {
                        x_incr = 1.0 * (x1 - x0) / abs(y1 - y);
                        alpha_incr = 1 / abs(y1 - y);
                    }
                    step_on_x = false;
                    x += x_incr * dy;
                    alpha += alpha_incr * dy;
                } else {
                    double dx = round(x) - x;
                    x = round(x);
                    x_incr = (x1 > x0) ? 1 : -1;
                    if (abs(x1 - x) < 0.1) {
                        y_incr = y1 - y0;
                        alpha_incr = 1;
                    } else {
                        y_incr = 1.0 * (y1 - y0) / abs(x1 - x);
                        alpha_incr = 1 / abs(x1 - x);
                    }
                    step_on_x = true;
                    y += y_incr * dx;
                    alpha += alpha_incr * dx;
                }
                bool break_flag;
                do {
                    break_flag = (step_on_x && round(x) != round(x1)) || (!step_on_x && round(y) != round(y1));

                    int segment_y = max(0, min(image_height - 1, (int) round(y)));
                    double segment_x = max(0.0, min(image_width - 1.0, x));

                    auto &segment = line_segments[segment_y];
                    bool not_found = segment.find(&shape) == segment.end();
                    auto &segment_it = segment[&shape];
                    if (not_found || segment_x < get<0>(segment_it)) {
                        get<0>(segment_it) = (int) round(segment_x);
                        get<1>(segment_it) = z;
                    }
                    if (not_found || segment_x > get<2>(segment_it)) {
                        get<2>(segment_it) = (int) round(segment_x);
                        get<3>(segment_it) = z;
                    }

                    x += x_incr;
                    y += y_incr;
                    alpha += alpha_incr;
                    z = intermediate_distance(alpha, z0, z1);
                } while (break_flag);
            }
        }
    }

//    ros::Time t1 = ros::Time::now();
//    t[0] = (t1 - t0).toSec();

    for (int y = 0; y < image_height; y++) {
//        ros::Time t2 = ros::Time::now();
        vector<segment_type> segments;
        if (line_segments[y].empty()) {
            continue;
        }
        for (auto segment : line_segments[y]) {
            segments.push_back(segment.second);
        }
//        ros::Time t3 = ros::Time::now();
//        t[1] += (t3 - t2).toSec();
        sort(segments.begin(), segments.end());
//        ros::Time t4 = ros::Time::now();
//        t[2] += (t4 - t3).toSec();

        list<segment_type *> segments_list;
        auto *cur_segment = &segments[0];
        int x_cur = get<0>(segments[0]);
        int x_next;
        double z = get<1>(segments[0]);
        int next_segment_idx = 0;
        bool bool_state = true;
        int segments_size = segments.size();

        while (true) {
            if (bool_state) {  // found another segment
                bool changed = false;
                if (get<1>(segments[next_segment_idx]) < z - 0.1) {
                    changed = true;
                } else if (abs(get<1>(segments[next_segment_idx]) - z) < 0.1) {
                    if (get<2>(segments[next_segment_idx]) < get<2>(*cur_segment)) {
                        changed = get<3>(segments[next_segment_idx])
                                  < intermediate_distance(get<2>(segments[next_segment_idx]), cur_segment);
                    } else {
                        changed = get<3>(*cur_segment)
                                  > intermediate_distance(get<2>(*cur_segment), &segments[next_segment_idx]);
                    }
                }
                if (changed) {
                    z = get<1>(segments[next_segment_idx]);
                    cur_segment = &segments[next_segment_idx];
                }

                segments_list.push_front(&segments[next_segment_idx]);
                next_segment_idx++;
            } else {  // ended segment
                cur_segment = nullptr;
                z = 1000000;
                for (auto it = segments_list.begin(); it != segments_list.end();) {
                    if (get<2>(**it) < x_cur) {
                        it = segments_list.erase(it);
                    } else {
                        double z_ = intermediate_distance(x_cur, *it);
                        if (z_ < z) {
                            z = z_;
                            cur_segment = *it;
                        }
                        ++it;
                    }
                }

                if (cur_segment == nullptr) {
                    if (next_segment_idx < segments_size) {
                        cur_segment = &segments[next_segment_idx];
                        segments_list.push_back(cur_segment);
                        x_cur = get<0>(*cur_segment);
                        bool_state = true;
                        continue;
                    } else {
                        break;
                    }
                }
            }

            if (next_segment_idx < segments_size
                && get<0>(segments[next_segment_idx])
                   < get<2>(*cur_segment) + 1) {
                x_next = get<0>(segments[next_segment_idx]);
                bool_state = true;
            } else {
                x_next = get<2>(*cur_segment) + 1;
                bool_state = false;
            }

//            ros::Time t7 = ros::Time::now();
            for (; x_cur < x_next; x_cur++) {
                save_point(x_cur, y, intermediate_distance(x_cur, cur_segment));
            }
//            ros::Time t8 = ros::Time::now();
//            t[6] += (t8 - t7).toSec();
        }
//        ros::Time t5 = ros::Time::now();
//        t[3] += (t5 - t4).toSec();
    }

    delete[] line_segments;

//    ros::Time t6 = ros::Time::now();
//    t[4] = (t6 - t1).toSec();
//    t[5] = (t6 - t0).toSec();
//    cout << "t[0] = " << t[0] << "\n";
//    cout << "t[1] = " << t[1] << "\n";
//    cout << "t[2] = " << t[2] << "\n";
//    cout << "t[3] = " << t[3] << "\n";
//    cout << "t[4] = " << t[4] << "\n";
//    cout << "t[5] = " << t[5] << "\n";
//    cout << "t[6] = " << t[6] << "\n";
}