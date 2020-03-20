#include <random>
#include "pointcloudTraj/map_generator.h"

using namespace std;

map_generator::map_generator(double x_init, double x_end, double y_init, double y_end,
                             double x1, double x2, double y1, double y2, double h1, double h2,
                             double w1, double w2, double res, int num, int seed) :
        x_init(x_init), x_end(x_end), y_init(y_init), y_end(y_end),
        x1(x1), x2(x2), y1(y1), y2(y2), h1(h1), h2(h2), w1(w1), w2(w2), res(res), num(num), seed(seed) {
    global_map_pcl = new pcl::PointCloud<pcl::PointXYZ>();
    shapes = new vector<vector<Eigen::Vector3d>>();
}

void map_generator::generate_map() {
    random_device rd;
    default_random_engine engine(seed != -1 ? seed : rd());

    auto random_x = uniform_real_distribution<double>(x1, x2);
    auto random_y = uniform_real_distribution<double>(y1, y2);
    auto random_h = uniform_real_distribution<double>(h1, h2);
    auto random_w = uniform_real_distribution<double>(w1, w2);
    double resolution = 0.1;

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

        vector<Eigen::Vector3d> shape_cap1;
        vector<Eigen::Vector3d> shape_cap2;
        vector<Eigen::Vector3d> shape_element;
        for (int delta = 90, phi = delta / 2; phi < 360 + delta / 2; phi += delta) {
            double x_1 = round((x + w * cos(M_PI / 180 * phi)) / resolution) * resolution;
            double y_1 = round((y + w * sin(M_PI / 180 * phi)) / resolution) * resolution;
            double x_2 = round((x + w * cos(M_PI / 180 * (phi + delta))) / resolution) * resolution;
            double y_2 = round((y + w * sin(M_PI / 180 * (phi + delta))) / resolution) * resolution;

            shape_element.clear();
            shape_element.emplace_back(x_1, y_1, 0);
            shape_element.emplace_back(x_2, y_2, 0);
            shape_element.emplace_back(x_2, y_2, h);
            shape_element.emplace_back(x_1, y_1, h);
            shapes->push_back(shape_element);

            shape_cap1.emplace_back(x_1, y_1, 0);
            shape_cap2.emplace_back(x_1, y_1, h);

            emplace_rect_to_map(x_1, y_1, x_2, y_2, h);
        }

        shapes->push_back(shape_cap1);
        shapes->push_back(shape_cap2);

        emplace_rect_to_map(shape_cap1[0][0], shape_cap1[0][1],
                            shape_cap1[2][0], shape_cap1[2][1], 0);
        emplace_rect_to_map(shape_cap1[0][0], shape_cap1[0][1],
                            shape_cap1[2][0], shape_cap1[2][1], h);
    }

    global_map_pcl->width = global_map_pcl->size();
    global_map_pcl->height = 1;
    global_map_pcl->is_dense = true;
}

const vector<vector<Eigen::Vector3d>> *map_generator::get_shapes() {
    return shapes;
}

const pcl::PointCloud<pcl::PointXYZ> *map_generator::get_global_map_pcl() {
    return global_map_pcl;
}

map_generator::~map_generator() {
    delete global_map_pcl;
    delete shapes;
}

void map_generator::emplace_rect_to_map(double x_1, double y_1, double x_2, double y_2, double h_) {
    int x_1_cell = (int) (x_1 / res);
    int y_1_cell = (int) (y_1 / res);
    int x_2_cell = (int) (x_2 / res);
    int y_2_cell = (int) (y_2 / res);
    int h_cell = (int) (h_ / res);

    int x_incr = (x_1_cell < x_2_cell) ? 1 : -1;
    int y_incr = (y_1_cell < y_2_cell) ? 1 : -1;
    if (x_1_cell == x_2_cell) {
        for (int y_i = y_1_cell; y_i != y_2_cell; y_i += y_incr) {
            for (int h_i = 1; h_i < h_cell; h_i++) {
                global_map_pcl->emplace_back(x_1_cell * res, y_i * res, h_i * res);
            }
        }
    } else if (y_1_cell == y_2_cell) {
        for (int x_i = x_1_cell; x_i != x_2_cell; x_i += x_incr) {
            for (int h_i = 1; h_i < h_cell; h_i++) {
                global_map_pcl->emplace_back(x_i * res, y_1_cell * res, h_i * res);
            }
        }
    } else {
        for (int x_i = x_1_cell; x_i != x_2_cell + x_incr; x_i += x_incr) {
            for (int y_i = y_1_cell; y_i != y_2_cell + y_incr; y_i += y_incr) {
                global_map_pcl->emplace_back(x_i * res, y_i * res, h_cell * res);
            }
        }
    }
}