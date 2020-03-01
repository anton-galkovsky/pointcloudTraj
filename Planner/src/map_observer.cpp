#include <cmath>
#include <iostream>
#include "pointcloudTraj/map_observer.h"


triangle_shape::triangle_shape(const std::vector<Eigen::Vector3f> &sp_points) {
    assert(sp_points.size() == 3);
    spatial_points = sp_points;
    normal = (spatial_points[1] - spatial_points[0]).cross(spatial_points[2] - spatial_points[1]);
}

int triangle_shape::get_right_border(int y) {
    double x = -100000;
    for (int i = 0; i < 3; i++) {
        int next_i = (i + 1) % 3;
        if ((image_points[next_i][1] - y) * (image_points[i][1] - y) <= 0) {
            x = std::max(x, image_points[i][0] + 1.0 * (y - image_points[i][1])
                / (image_points[next_i][1] - image_points[i][1]) * (image_points[next_i][0] - image_points[i][0]));
        }
    }
    return (int) round(x);
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
        shapes.emplace_back(triangle_shape(shape));
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
    auto *object_stacks = new std::vector<std::pair<triangle_shape *, double>>[image_width * image_height];

    for (auto &shape : shapes) {
        if (shape.normal.dot(camera_axis_z) < 0) {
            for (const auto& point : shape.spatial_points) {
                Eigen::Vector3f delta = point - camera_translation;
                float distance = delta.norm();
                float scale = focal_distance / distance;
                int img_x = (int) round(delta.dot(camera_axis_x) * scale * image_width + image_width / 2.0);
                int img_y = (int) round(delta.dot(camera_axis_y) * scale * image_width + image_height / 2.0);
                shape.image_points.emplace_back(Eigen::Vector3f(img_x, img_y, distance));                               // outside screen?
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
                if (abs(x1 - x0) < abs(y1 - y0)) {
                    x_incr = 1.0 * (x1 - x0) / abs(y1 - y0);
                    y_incr = (y1 > y0) ? 1 : -1;
                    z_incr = (shape.image_points[(i + 1) % image_points_size][2] - z) / abs(y1 - y0);
                    while (y != y1) {
                        object_stacks[(int) (round(y) * image_width + round(x))]
                                .emplace_back(std::make_pair(&shape, z));
                        x += x_incr;
                        y += y_incr;
                        z += z_incr;
                    }
                } else {
                    x_incr = (x1 > x0) ? 1 : -1;
                    y_incr = 1.0 * (y1 - y0) / abs(x1 - x0);
                    z_incr = (shape.image_points[(i + 1) % image_points_size][2] - z) / abs(x1 - x0);
                    while (x != x1) {
                        object_stacks[(int) (round(y) * image_width + round(x))]
                                .emplace_back(std::make_pair(&shape, z));
                        x += x_incr;
                        y += y_incr;
                        z += z_incr;
                    }
                }
            }
        }
    }

    triangle_shape *cur = nullptr;
    double z_cur = 0, z_incr = 0;
    int x_delta = 0;
    for (int idx = 0; idx < image_width * image_height; idx++) {
        if (cur == nullptr && object_stacks[idx].empty()) {
            image[idx] = INFINITY;
        } else if (cur == nullptr && !object_stacks[idx].empty()) {
            cur = object_stacks[idx][0].first;
            int right_border = cur->get_right_border(idx / image_width);
            x_delta = right_border - idx % image_width;
            while (right_border < image_width - 1 && !object_stacks[idx + x_delta + 1].empty()
                   && object_stacks[idx + x_delta + 1][0].first == cur) {
                right_border++;
                x_delta++;
            }
            if (x_delta != 0) {
                z_incr = (object_stacks[idx + x_delta][0].second - object_stacks[idx][0].second) / x_delta;
                z_cur = object_stacks[idx][0].second;
                image[idx] = z_cur;
                z_cur += z_incr;
                x_delta--;
            } else {
                image[idx] = object_stacks[idx][0].second;
                cur = nullptr;
            }
        } else if (cur != nullptr && x_delta == 0) {
            assert(object_stacks[idx][0].first == cur);
            image[idx] = z_cur;
            cur = nullptr;
        } else if (cur != nullptr) {
            image[idx] = z_cur;
            z_cur += z_incr;
            x_delta--;
        } else {
            assert(1 == 0);
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

//    for (auto shape : shapes)
//        for (auto point : shape.image_points) {
//            image[point[1] * image_width + point[0]] = 1;
//        }
    return image;
}