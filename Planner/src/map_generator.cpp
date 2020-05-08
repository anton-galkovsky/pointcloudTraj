#include <random>
#include <algorithm>
#include "pointcloudTraj/map_generator.h"

using namespace std;

map_generator::map_generator(double x_init, double x_end, double y_init, double y_end,
                             double x1, double x2, double y1, double y2, double h1, double h2,
                             double w1, double w2, double res, int num, int seed) :
        x_init(x_init), x_end(x_end), y_init(y_init), y_end(y_end),
        x1(x1), x2(x2), y1(y1), y2(y2), h1(h1), h2(h2), w1(w1), w2(w2), res(res), num(num), seed(seed) {
    global_map_pcl.height = 1;
    global_map_pcl.is_dense = true;
}

void map_generator::generate_map() {
    shapes.clear();
    global_map_pcl.clear();

    random_device rd;
    default_random_engine engine(seed != -1 ? seed : rd());

    uniform_real_distribution<double> random_x(x1, x2);
    uniform_real_distribution<double> random_y(y1, y2);
    uniform_real_distribution<double> random_h(h1, h2);
    uniform_real_distribution<double> random_w(w1, w2);

    vector<Eigen::Vector3d> cylinders;
    for (int i = 0; i < num; i++) {
        double x, y, w, h;
        x = random_x(engine);
        y = random_y(engine);
        w = random_w(engine);
        h = random_h(engine);

        if (pow(x - x_init, 2) + pow(y - y_init, 2) < 4
            || pow(x - x_end, 2) + pow(y - y_end, 2) < 4) {
            continue;
        }
        bool ok = true;
        for (auto cylinder : cylinders) {
            if (pow(cylinder[0] - x, 2) + pow(cylinder[1] - y, 2) < pow(cylinder[2] + w, 2)) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            continue;
        }
        cylinders.emplace_back(x, y, w);

        h = round(h / res) * res;

        vector<Eigen::Vector3d> shape_cap1;
        vector<Eigen::Vector3d> shape_cap2;
        vector<Eigen::Vector3d> shape_element;
        for (int delta = 90, phi = delta / 2; phi < 360 + delta / 2; phi += delta) {
            double x_1 = round((x + w * cos(M_PI / 180 * phi)) / res) * res;
            double y_1 = round((y + w * sin(M_PI / 180 * phi)) / res) * res;
            double x_2 = round((x + w * cos(M_PI / 180 * (phi + delta))) / res) * res;
            double y_2 = round((y + w * sin(M_PI / 180 * (phi + delta))) / res) * res;

            shape_element.clear();
            shape_element.emplace_back(x_1, y_1, 0);
            shape_element.emplace_back(x_2, y_2, 0);
            shape_element.emplace_back(x_2, y_2, h);
            shape_element.emplace_back(x_1, y_1, h);
            shapes.push_back(shape_element);

            shape_cap1.emplace_back(x_1, y_1, 0);
            shape_cap2.emplace_back(x_1, y_1, h);

            emplace_rect_to_map(x_1, y_1, x_2, y_2, h);
        }

        emplace_rect_to_map(shape_cap1[0][0], shape_cap1[0][1],
                            shape_cap1[2][0], shape_cap1[2][1], 0);
        emplace_rect_to_map(shape_cap1[0][0], shape_cap1[0][1],
                            shape_cap1[2][0], shape_cap1[2][1], h);

        reverse(shape_cap1.begin(), shape_cap1.end());
        shapes.push_back(shape_cap1);
        shapes.push_back(shape_cap2);
    }

    global_map_pcl.width = global_map_pcl.size();
}

vector<vector<Eigen::Vector3d>> map_generator::get_shapes() {
    return shapes;
}

pcl::PointCloud<pcl::PointXYZ> map_generator::get_global_map_pcl() {
    return global_map_pcl;
}

void map_generator::emplace_rect_to_map(double x_1, double y_1, double x_2, double y_2, double h_) {
    int x_1_cell = (int) round(x_1 / res);
    int y_1_cell = (int) round(y_1 / res);
    int x_2_cell = (int) round(x_2 / res);
    int y_2_cell = (int) round(y_2 / res);
    int h_cell   = (int) round(h_  / res);

    int x_incr = (x_1_cell < x_2_cell) ? 1 : -1;
    int y_incr = (y_1_cell < y_2_cell) ? 1 : -1;
    if (x_1_cell == x_2_cell) {
        for (int y_i = y_1_cell; y_i != y_2_cell; y_i += y_incr) {
            for (int h_i = 1; h_i < h_cell; h_i++) {
                global_map_pcl.emplace_back(x_1_cell * res, y_i * res, h_i * res);
            }
        }
    } else if (y_1_cell == y_2_cell) {
        for (int x_i = x_1_cell; x_i != x_2_cell; x_i += x_incr) {
            for (int h_i = 1; h_i < h_cell; h_i++) {
                global_map_pcl.emplace_back(x_i * res, y_1_cell * res, h_i * res);
            }
        }
    } else {
        for (int x_i = x_1_cell; x_i != x_2_cell + x_incr; x_i += x_incr) {
            for (int y_i = y_1_cell; y_i != y_2_cell + y_incr; y_i += y_incr) {
                global_map_pcl.emplace_back(x_i * res, y_i * res, h_cell * res);
            }
        }
    }
}


marked_map_generator::marked_map_generator(double x_init, double x_end, double y_init, double y_end,
                                           double x1, double x2, double y1, double y2, double h1, double h2,
                                           double w1, double w2, double res, double density, int num, int seed) :
        map_generator(x_init, x_end, y_init, y_end, x1, x2, y1, y2, h1, h2, w1, w2, res, num, seed),
        res(res), density(density), seed(seed) {
}

void marked_map_generator::generate_map() {
    map_generator::generate_map();

    voxel_value_map marked_points_voxel_map(res);

    marked_point_indexes_arr.clear();
    for (const auto &shape_element : shapes) {
        emplace_marked_rect(shape_element, marked_points_voxel_map);
    }
    marked_map_pcl = marked_points_voxel_map.get_voxel_cloud();
    marked_map_pcl.height = 1;
    marked_map_pcl.is_dense = true;
    marked_map_pcl.width = marked_map_pcl.size();
}

vector<vector<int>> marked_map_generator::get_marked_point_indexes_arr() {
    return marked_point_indexes_arr;
}

pcl::PointCloud<pcl::PointXYZ> marked_map_generator::get_marked_map_pcl() {
    return marked_map_pcl;
}

void marked_map_generator::emplace_marked_rect(const vector<Eigen::Vector3d> &shape, voxel_value_map &marked_map) {
    vector<Eigen::Vector3d> shape_marked_points;

    random_device rd;
    default_random_engine engine(seed != -1 ? seed : rd());
    uniform_real_distribution<double> random_a(0, 1);
    uniform_real_distribution<double> random_b(0, 1);

    Eigen::Vector3d v_a = shape[0] - shape[1];
    Eigen::Vector3d v_b = shape[2] - shape[1];
    Eigen::Vector3d v_c = shape[1];

    double square = v_a.norm() / res * v_b.norm() / res;
    int marked_num = (int) (square * density);

    shape_marked_points.emplace_back(v_c + v_a * 0 + v_b * 0);
    shape_marked_points.emplace_back(v_c + v_a * 0 + v_b * 1);
    shape_marked_points.emplace_back(v_c + v_a * 1 + v_b * 0);
    shape_marked_points.emplace_back(v_c + v_a * 1 + v_b * 1);
    for (int i = 0; i < marked_num; i++) {
        double a = random_a(engine);
        double b = random_b(engine);
        shape_marked_points.emplace_back(v_c + v_a * a + v_b * b);
    }

    vector<int> shape_marked_voxel_indexes;
    for (const auto& point : shape_marked_points) {
        int index = marked_map.add_point(point);
        shape_marked_voxel_indexes.push_back(index);
    }
    marked_point_indexes_arr.push_back(shape_marked_voxel_indexes);
}