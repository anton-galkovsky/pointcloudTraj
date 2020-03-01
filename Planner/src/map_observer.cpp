#include <cmath>
#include <tuple>
#include <list>
#include <algorithm>
#include <iostream>
#include "pointcloudTraj/map_observer.h"


triangle_shape::triangle_shape(const std::vector<Eigen::Vector3f> &sp_points) {
    assert(sp_points.size() == 3);
    spatial_points = sp_points;
    normal = (spatial_points[1] - spatial_points[0]).cross(spatial_points[2] - spatial_points[1]);
}


map_observer::map_observer(const std::vector<std::vector<Eigen::Vector3f>> &shapes_,
                           int img_width, int img_height, int fov_hor) :
        image_width(img_width), image_height(img_height) {
    focal_distance = (float) (0.5 / tan(fov_hor / 2.0 * M_PI / 180));
    camera_translation = Eigen::Vector3f(0, 0, 0);
    camera_axis_x = Eigen::Vector3f(1, 0, 0);
    camera_axis_y = Eigen::Vector3f(0, 1, 0);
    camera_axis_z = Eigen::Vector3f(0, 0, 1);
    for (const auto &shape : shapes_) {
        shapes.emplace_back(shape);
    }
}

void map_observer::set_camera_pose(const Eigen::Affine3f &camera_pose) {
    camera_translation = camera_pose.translation();
    camera_axis_x = camera_pose.rotation() * Eigen::Vector3f(1, 0, 0);
    camera_axis_y = camera_pose.rotation() * Eigen::Vector3f(0, 1, 0);
    camera_axis_z = camera_pose.rotation() * Eigen::Vector3f(0, 0, 1);
}

float *map_observer::render_to_image() {
    auto *image = new float[image_width * image_height];
    auto *line_segments = new std::map<triangle_shape *, float[4]>[image_height];

    for (auto &shape : shapes) {
        if (shape.normal.dot(camera_axis_z) < 0) {
            for (const auto& point : shape.spatial_points) {
                Eigen::Vector3f delta = point - camera_translation;
                float distance = delta.norm();
                float scale = focal_distance / distance;
                int img_x = (int) round(delta.dot(camera_axis_x) * scale * (float) image_width
                                        + image_width / 2.0);
                int img_y = (int) round(delta.dot(camera_axis_y) * scale * (float) image_width
                                        + image_height / 2.0);
                shape.image_points.emplace_back(img_x, img_y, distance);                                                // outside screen?
//                std::cout << "point: (" << img_x << ", " << img_y << ", " << distance << ")\n";
            }
            int image_points_size = shape.image_points.size();
            for (int i = 0; i < image_points_size; i++) {
                int x0 = shape.image_points[i][0];
                int y0 = shape.image_points[i][1];
                int x1 = shape.image_points[(i + 1) % image_points_size][0];
                int y1 = shape.image_points[(i + 1) % image_points_size][1];
                double x_incr, y_incr, z_incr;
                double x = x0;
                double y = y0;
                double z = shape.image_points[i][2];
                bool step_on_x;
                if (abs(x1 - x0) < abs(y1 - y0)) {
                    x_incr = 1.0 * (x1 - x0) / abs(y1 - y0);
                    y_incr = (y1 > y0) ? 1 : -1;
                    z_incr = (shape.image_points[(i + 1) % image_points_size][2] - z) / abs(y1 - y0);
                    step_on_x = false;
                } else {
                    x_incr = (x1 > x0) ? 1 : -1;
                    y_incr = 1.0 * (y1 - y0) / abs(x1 - x0);
                    z_incr = (shape.image_points[(i + 1) % image_points_size][2] - z) / abs(x1 - x0);
                    step_on_x = true;
                }
                while ((step_on_x && x != x1) || (!step_on_x && y != y1)) {
                    auto &segm = line_segments[(int) round(y)];
                    if (segm.find(&shape) == segm.end()) {
                        auto &segm_it = segm[&shape];
                        segm_it[0] = segm_it[2] = (float) round(x);
                        segm_it[1] = segm_it[3] = (float) z;
                    } else {
                        auto &segm_it = segm[&shape];
                        if (x < segm_it[0]) {
                            segm_it[0] = (float) round(x);
                            segm_it[1] = (float) z;
                        }
                        if (x > segm_it[2]) {
                            segm_it[2] = (float) round(x);
                            segm_it[3] = (float) z;
                        }
                    }

                    x += x_incr;
                    y += y_incr;
                    z += z_incr;
                }
            }
        }
    }

    for (int y = 0; y < image_height; y++) {
//        std::cout << "\n" << y;
        std::vector<std::tuple<int, float, int, float>> segments;
        if (line_segments[y].empty()) {
            continue;
        }
        for (auto segment : line_segments[y]) {
            segments.emplace_back(segment.second[0], segment.second[1], segment.second[2], segment.second[3]);
        }
        std::sort(segments.begin(), segments.end());


        std::list<std::tuple<int, float, int, float> *> segments_list;
        auto *cur_segment = &segments[0];
        segments_list.push_front(cur_segment);
        int x_cur = std::get<0>(segments[0]);                                                      //really this?
        int x_next;
        float z = std::get<1>(segments[0]);
        float z_incr = 0;
        int next_segment_idx = 1;
        int state = 0;
        int segments_size = segments.size();
        bool ended = false;

        while (!ended) {                                                                                // if size == 1?
//            std::cout << state;
            switch (state) {
                case 0:  // just started
                    if (next_segment_idx < segments_size
                        && std::get<0>(segments[next_segment_idx])
                           < std::get<2>(*cur_segment) + 1) {
                        x_next = std::get<0>(segments[next_segment_idx]);
                        state = 1;
                    } else {
                        x_next = std::get<2>(*cur_segment) + 1;
                        state = 2;
                    }

                    z_incr = (std::get<3>(segments[next_segment_idx - 1]) - z)
                             / (float) (std::get<2>(segments[next_segment_idx - 1]) - x_cur);
                    for (; x_cur < x_next; x_cur++) {
                        image[y * image_width + x_cur] = z;
                        z += z_incr;
                    }
                    break;
                case 1: { // found another segment
                    bool changed = std::get<1>(segments[next_segment_idx]) < z;
                    if (changed) {
                        z = std::get<1>(segments[next_segment_idx]);
                        cur_segment = &segments[next_segment_idx];
                    }
                    segments_list.push_front(&segments[next_segment_idx]);
                    next_segment_idx++;

                    if (next_segment_idx < segments_size
                        && std::get<0>(segments[next_segment_idx])
                           < std::get<2>(*cur_segment) + 1) {
                        x_next = std::get<0>(segments[next_segment_idx]);
                        state = 1;
                    } else {
                        x_next = std::get<2>(*cur_segment) + 1;
                        state = 2;
                    }

                    if (changed) {
                        z_incr = (std::get<3>(segments[next_segment_idx - 1]) - z)
                                 / (float) (std::get<2>(segments[next_segment_idx - 1]) - x_cur);
                    }

                    for (; x_cur < x_next; x_cur++) {
                        image[y * image_width + x_cur] = z;
                        z += z_incr;
                    }
                }
                    break;
                case 2: { // ended segment
                    cur_segment = nullptr;
                    z = 1000000;
                    for (auto it = segments_list.begin(); it != segments_list.end(); ++it) {
                        if (std::get<2>(**it) < x_cur) {
                            it = segments_list.erase(it);
                        } else {
                            float z_incr_ = (std::get<3>(**it) - std::get<1>(**it))
                                            / (float) (std::get<2>(**it) - std::get<0>(**it));
                            float z_ = std::get<1>(**it) + (float) (x_cur - std::get<0>(**it)) * z_incr_;
                            if (z_ < z) {
                                z = z_;
                                z_incr = z_incr_;
                                cur_segment = *it;
                            }
                        }
                    }

                    if (cur_segment != nullptr) {
                        if (next_segment_idx < segments_size
                            && std::get<0>(segments[next_segment_idx])
                               < std::get<2>(*cur_segment) + 1) {
                            x_next = std::get<0>(segments[next_segment_idx]);
                            state = 1;
                        } else {
                            x_next = std::get<2>(*cur_segment) + 1;
                            state = 2;
                        }

                        for (; x_cur < x_next; x_cur++) {
                            image[y * image_width + x_cur] = z;
                            z += z_incr;
                        }
                    } else {
                        if (next_segment_idx < segments_size) {
                            cur_segment = &segments[next_segment_idx];
                            segments_list.push_back(cur_segment);
                            x_cur = std::get<0>(*cur_segment);
                            state = 1;
                        } else {
                            ended = true;
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < image_width; i++) {
        for (int j = 0; j < image_height; j++) {
            if (image[j * image_width + i] < 0.001) {
                image[j * image_width + i] = INFINITY;
            }
        }
    }

//    FILE *fout = fopen("/home/galanton/catkin_ws/view__txt", "w");
//    for (int i = 0; i < image_width; i++) {
//        for (int j = 0; j < image_height; j++) {
//            fprintf(fout, "%7.2f", image[j * image_width + i]);
//        }
//        fprintf(fout, "\n");
//    }
//    fclose(fout);

    return image;
}