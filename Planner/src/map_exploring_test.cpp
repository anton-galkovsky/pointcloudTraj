#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "pointcloudTraj/map_observer.h"
#include "pointcloudTraj/map_generator.h"
#include "pointcloudTraj/voxel_map.h"

using namespace std;

static marked_map_observer *observer;

static double focal_distance;
static double res;
static int image_w, image_h;
static double x1_init, y1_init, z1_init;
static double x2_init, y2_init, z2_init;
static double x3_init, y3_init, z3_init;
static double axis_z1_0, axis_z1_1, axis_z1_2;
static double axis_z2_0, axis_z2_1, axis_z2_2;
static double axis_z3_0, axis_z3_1, axis_z3_2;

typedef tuple<const Eigen::Vector3d *, int, int> marked_img_pt_type;
typedef tuple<const Eigen::Vector3d *, int, int, int, int> marked_img_pt_pair_type;
typedef tuple<const Eigen::Vector3d *, Eigen::Vector3f, Eigen::Vector3f> marked_diagonal_type;
typedef tuple<Eigen::Vector3f, Eigen::Vector3f, double, double> marked_cone_value_type;

Eigen::Vector3f to_vec_3d(double x, double y, const Eigen::Matrix3f &rotation) {
    return (x / image_w - 0.5) / focal_distance * rotation.col(0) +
           (y - 0.5 * image_h) / image_w / focal_distance * rotation.col(1) +
           rotation.col(2);
}

double to_x_comp(double x) {
    return (x / image_w - 0.5) / focal_distance;
}

double to_y_comp(double y) {
    return (y - 0.5 * image_h) / image_w / focal_distance;
}

void intersect_points_with_line(list<Eigen::Vector2f> &points, const Eigen::Vector2f &normal) {
    for (auto it_1 = points.begin(), it = it_1; it != points.end();) {
        ++it_1;
        if (it_1 == points.end()) {
            it_1 = points.begin();
        }
        double n_dot_i = it->dot(normal);
        double n_dot_i_1 = it_1->dot(normal);
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
        if (it->dot(normal) < -0.01) {
            it = points.erase(it);
        } else {
            ++it;
        }
    }
}

void intersect_2d_cones(const Eigen::Vector2f &n_1_r, const Eigen::Vector2f &n_1_l,
                        const Eigen::Vector2f &v_2_r, const Eigen::Vector2f &v_2_l, const Eigen::Vector2f &t_2_x,
                        double &min_x, double &min_z, double &max_x, double &max_z) {
    list<Eigen::Vector2f> points;
    points.push_back(t_2_x);
    points.emplace_back(t_2_x + v_2_r * 100);
    points.emplace_back(t_2_x + v_2_l * 100);

    intersect_points_with_line(points, n_1_r);
    intersect_points_with_line(points, n_1_l);

    min_x = max_x = (*points.begin())[0];
    min_z = max_z = (*points.begin())[1];
    for (auto it = ++points.begin(); it != points.end(); ++it) {
        double z = (*it)[1];
        if (min_z > z) {
            min_z = z;
            min_x = (*it)[0];
        }
        if (max_z < z) {
            max_z = z;
            max_x = (*it)[0];
        }
    }
}

void add_diagonal(const marked_img_pt_pair_type &p, const Eigen::Affine3f &pose_1,
                  const Eigen::Matrix3f &rotation_2, const Eigen::Vector3f &t_2,
                  vector<marked_diagonal_type> &cone_arrows) {
//    Eigen::Matrix3f rotation_2 = pose_1.rotation().inverse() * pose_2.rotation();
//    Eigen::Vector3f translation_2 = pose_2.translation() - pose_1.translation();
//    Eigen::Vector3f t_2 = pose_1.rotation().inverse() * translation_2;

    int x_1 = get<1>(p);
    int y_1 = get<2>(p);
    int x_2 = get<3>(p);
    int y_2 = get<4>(p);

    double x_1_m = to_x_comp(x_1);
    double x_1_r = to_x_comp(x_1 + 0.5);
    double x_1_l = to_x_comp(x_1 - 0.5);
    double y_1_m = to_y_comp(y_1);
    double y_1_u = to_y_comp(y_1 - 0.5);
    double y_1_d = to_y_comp(y_1 + 0.5);
    Eigen::Vector2f n_1_r(-1, x_1_r);
    Eigen::Vector2f n_1_l(1, -x_1_l);
    Eigen::Vector2f n_1_u(1, -y_1_u);
    Eigen::Vector2f n_1_d(-1, y_1_d);

    Eigen::Vector3f v_2_ru = to_vec_3d(x_2 + 0.5, y_2 - 0.5, rotation_2);
    Eigen::Vector3f v_2_rd = to_vec_3d(x_2 + 0.5, y_2 + 0.5, rotation_2);
    Eigen::Vector3f v_2_lu = to_vec_3d(x_2 - 0.5, y_2 - 0.5, rotation_2);
    Eigen::Vector2f v_2_r(v_2_ru[0], v_2_ru[2]);
    Eigen::Vector2f v_2_l(v_2_lu[0], v_2_lu[2]);
    Eigen::Vector2f v_2_u(v_2_ru[1], v_2_lu[2]);
    Eigen::Vector2f v_2_d(v_2_rd[1], v_2_rd[2]);
    Eigen::Vector2f t_2_x(t_2[0], t_2[2]);
    Eigen::Vector2f t_2_y(t_2[1], t_2[2]);

    double min_x, min_z, max_x, max_z, min_y, min_z_, max_y, max_z_;
    intersect_2d_cones(n_1_r, n_1_l, v_2_r, v_2_l, t_2_x, min_x, min_z, max_x, max_z);
    intersect_2d_cones(n_1_u, n_1_d, v_2_u, v_2_d, t_2_y, min_y, min_z_, max_y, max_z_);

    Eigen::Vector3f p_min, p_max;

    if (min_z < min_z_) {
        p_min = Eigen::Vector3f(min_z < 0.01 ? x_1_m * min_z_ : min_x * min_z_ / min_z, min_y, min_z_);
    } else {
        p_min = Eigen::Vector3f(min_x, min_z_ < 0.01 ? y_1_m * min_z : min_y * min_z / min_z_, min_z);
    }
    if (max_z > max_z_) {
        p_max = Eigen::Vector3f(max_z < 0.01 ? x_1_m * max_z_ : max_x * max_z_ / max_z, max_y, max_z_);
    } else {
        p_max = Eigen::Vector3f(max_x, max_z_ < 0.01 ? y_1_m * max_z : max_y * max_z / max_z_, max_z);
    }

    p_min = pose_1 * p_min;
    p_max = pose_1 * p_max;

    cone_arrows.emplace_back(get<0>(p), p_min, p_max);
}

struct cone {
    cone(int x, int y, const Eigen::Affine3f &pose) :
            c(pose.translation()) {
        Eigen::Vector3f v1, v2, v3, v4;
        v_ru = to_vec_3d(x + 0.5, y - 0.5, pose.rotation()) * 100;
        v_rd = to_vec_3d(x + 0.5, y + 0.5, pose.rotation()) * 100;
        v_ld = to_vec_3d(x - 0.5, y + 0.5, pose.rotation()) * 100;
        v_lu = to_vec_3d(x - 0.5, y - 0.5, pose.rotation()) * 100;
        n_r = v_ru.cross(v_rd).normalized();
        n_d = v_rd.cross(v_ld).normalized();
        n_l = v_ld.cross(v_lu).normalized();
        n_u = v_lu.cross(v_ru).normalized();
    }

    Eigen::Vector3f c;
    Eigen::Vector3f v_ru;
    Eigen::Vector3f v_rd;
    Eigen::Vector3f v_ld;
    Eigen::Vector3f v_lu;
    Eigen::Vector3f n_u;
    Eigen::Vector3f n_d;
    Eigen::Vector3f n_r;
    Eigen::Vector3f n_l;
};

template<class T>
void intersect_points_with_plane(list<T> &points, const T &plane_point, const T &normal) {
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

void intersect_points_with_cone(list<Eigen::Vector3f> &points, const cone &cone) {
    intersect_points_with_plane(points, cone.c, cone.n_r);
    intersect_points_with_plane(points, cone.c, cone.n_l);
    intersect_points_with_plane(points, cone.c, cone.n_u);
    intersect_points_with_plane(points, cone.c, cone.n_d);
}

void add_all_lines(vector<Eigen::Vector3f> &points, voxel_map_pcl &line_voxels, visualization_msgs::Marker &line_list) {
    geometry_msgs::Point p2;
    geometry_msgs::Point p1;

    for (int i = 0; i < (int) points.size() - 1; i++) {
        for (int j = i + 1; j < (int) points.size(); j++) {
            p1.x = points[i][0];
            p1.y = points[i][1];
            p1.z = points[i][2];
            p2.x = points[j][0];
            p2.y = points[j][1];
            p2.z = points[j][2];
            line_list.points.push_back(p1);
            line_list.points.push_back(p2);


            Eigen::Vector3f dv = points[j] - points[i];
            vector<Eigen::Vector3d> pts;
            double dv_norm = dv.norm();
            dv = dv / dv_norm * 0.1;
            for (int k = 0; k <= dv_norm / 0.1; k++) {
                Eigen::Vector3f v = points[i] + dv * k;
                pts.emplace_back(v[0], v[1], v[2]);
            }
            line_voxels.add_point_cloud(pts);
        }
    }
}

void intersect_cones(const cone &cone_1, const cone &cone_2, voxel_map_pcl &line_voxels,
                     visualization_msgs::Marker &line_list) {
    vector<Eigen::Vector3f> points;

    list<Eigen::Vector3f> points_u;
    points_u.emplace_back(cone_1.c);
    points_u.emplace_back(cone_1.v_ru + cone_1.c);
    points_u.emplace_back(cone_1.v_lu + cone_1.c);
    intersect_points_with_cone(points_u, cone_2);
    for (const auto &p : points_u) {
        points.push_back(p);
    }

    list<Eigen::Vector3f> points_d;
    points_d.emplace_back(cone_1.c);
    points_d.emplace_back(cone_1.v_rd + cone_1.c);
    points_d.emplace_back(cone_1.v_ld + cone_1.c);
    intersect_points_with_cone(points_d, cone_2);
    for (const auto &p : points_d) {
        points.push_back(p);
    }

    list<Eigen::Vector3f> points_l;
    points_l.emplace_back(cone_1.c);
    points_l.emplace_back(cone_1.v_lu + cone_1.c);
    points_l.emplace_back(cone_1.v_ld + cone_1.c);
    intersect_points_with_cone(points_l, cone_2);
    for (const auto &p : points_l) {
        points.push_back(p);
    }

    list<Eigen::Vector3f> points_r;
    points_r.emplace_back(cone_1.c);
    points_r.emplace_back(cone_1.v_ru + cone_1.c);
    points_r.emplace_back(cone_1.v_rd + cone_1.c);
    intersect_points_with_cone(points_r, cone_2);
    for (const auto &p : points_r) {
        points.push_back(p);
    }

    points = voxel_map_vec_f::to_voxel_cloud(points, 0.01);

    add_all_lines(points, line_voxels, line_list);
}

vector<marked_img_pt_pair_type> intersect_sorted_marked_img_pts(
        const vector<marked_img_pt_type> &marked_img_pts_1,
        const vector<marked_img_pt_type> &marked_img_pts_2) {
    vector<marked_img_pt_pair_type> intersection;
    auto it_1 = marked_img_pts_1.begin();
    auto it_2 = marked_img_pts_2.begin();
    while (it_1 != marked_img_pts_1.end() && it_2 != marked_img_pts_2.end()) {
        if (get<0>(*it_1) < get<0>(*it_2)) {
            ++it_1;
        } else if (get<0>(*it_1) > get<0>(*it_2)) {
            ++it_2;
        } else {
            intersection.emplace_back(get<0>(*it_1),
                                      get<1>(*it_1), get<2>(*it_1), get<1>(*it_2), get<2>(*it_2));
            ++it_1;
            ++it_2;
        }
    }
    return intersection;
}

void intersect_diagonals(vector<marked_diagonal_type> &cone_arrows,
                         const vector<marked_img_pt_type> &marked_img_pts, const Eigen::Affine3f &camera_pose) {
    auto it_1 = cone_arrows.begin();
    auto it_2 = marked_img_pts.begin();
    while (it_1 != cone_arrows.end() && it_2 != marked_img_pts.end()) {
        if (get<0>(*it_1) < get<0>(*it_2)) {
            ++it_1;
        } else if (get<0>(*it_1) > get<0>(*it_2)) {
            ++it_2;
        } else {
            Eigen::Vector3f P = camera_pose.translation();
            Eigen::Vector3f C = get<1>(*it_1);
            Eigen::Vector3f D = get<2>(*it_1);
            Eigen::Vector3f a = D - C;
            Eigen::Vector3f v = to_vec_3d(get<1>(*it_2), get<2>(*it_2), camera_pose.rotation());
            Eigen::Vector3f r = a.cross(v);
            double r_norm = r.norm();

            if (r_norm > 0.001) {
                Eigen::Vector3f g = v.cross(r);
                double a_norm = a.norm();
                double v_norm = v.norm();

                double cylinder_rad = 0.1;
                double ctg = tan(M_PI_2 - asin(r_norm / (a_norm * v_norm)));
                double delta = cylinder_rad * ctg / a_norm;
                double base = (P - C).dot(g) / (r_norm * r_norm);

                Eigen::Vector3f L = C + base * a;
                Eigen::Vector3f dL = delta * a;
                Eigen::Vector3f L1 = L + dL;
                Eigen::Vector3f L2 = L - dL;

                if ((L1 - C).dot(a) > 0 && (L1 - D).dot(a) < 0) {
                    get<1>(*it_1) = L1;
                }
                if ((L2 - C).dot(a) > 0 && (L2 - D).dot(a) < 0) {
                    get<2>(*it_1) = L2;
                }
            }

            ++it_1;
            ++it_2;
        }
    }
}

Eigen::Matrix3f compute_rotation(double axis_z_0, double axis_z_1, double axis_z_2) {
    Eigen::Vector3f camera_axis_z, camera_axis_x, camera_axis_y;
    camera_axis_z << axis_z_0, axis_z_1, axis_z_2;
    camera_axis_z /= camera_axis_z.norm();

    camera_axis_x = camera_axis_z.cross(Eigen::Vector3f(0, 0, 1));

    camera_axis_y = camera_axis_z.cross(camera_axis_x);
    if (camera_axis_y[2] > 0) {
        camera_axis_x *= -1;
        camera_axis_y *= -1;
    }
    Eigen::Matrix3f rotation;
    rotation << camera_axis_x, camera_axis_y, camera_axis_z;
    return rotation;
}

void append_marker_array_msg(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2,
                             double r, double g, double b, double x, double y, bool pointer,
                             visualization_msgs::MarkerArray &marker_array_msg) {
    static int id = 0;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "/arrows";
    arrow.id = id++;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = x;
    arrow.scale.y = y;
    if (pointer) {
        arrow.scale.z = 0.0;
    } else {
        arrow.scale.z = 0.01;
    }
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1;

    geometry_msgs::Point p;
    p.x = v1[0];
    p.y = v1[1];
    p.z = v1[2];
    arrow.points.push_back(p);
    p.x = v2[0];
    p.y = v2[1];
    p.z = v2[2];
    arrow.points.push_back(p);
    marker_array_msg.markers.push_back(arrow);
}

void fill_cone_arrows_msg(vector<marked_diagonal_type> &cone_arrows, visualization_msgs::MarkerArray &cone_arrows_msg,
                          double r, double g, double b, double w) {
    for (const auto &cone_arrow : cone_arrows) {
        append_marker_array_msg(get<1>(cone_arrow), get<2>(cone_arrow),
                                r, g, b, w, w, false, cone_arrows_msg);
    }
}

void add_arrow_marker(const Eigen::Affine3f &camera_pose, visualization_msgs::MarkerArray &z_arrows) {
    append_marker_array_msg(camera_pose.translation(), camera_pose.translation() + camera_pose.rotation().col(2),
                            0, 1, 0, 0.06, 0.1, true, z_arrows);
}

void shade_diagonals(vector<marked_diagonal_type> &cone_arrows, sensor_msgs::PointCloud2 &arrow_voxels_msg) {
//    voxel_map_pcl arrow_voxels(res);
    pcl::PointCloud<pcl::PointXYZ> arrow_voxels_pcl;
    for (const auto &cone_arrow : cone_arrows) {
        Eigen::Vector3f dv = get<2>(cone_arrow) - get<1>(cone_arrow);
        int steps = (int) round(dv.maxCoeff() / res);
        dv /= steps;
        for (int k = 0; k <= steps; k++) {
//            arrow_voxels.add_point(Eigen::Vector3f(cone_arrows[i] + k * dv));

            Eigen::Vector3f v(get<1>(cone_arrow) + k * dv);
            pcl::PointXYZ p;
            p.x = round(v[0] / (float) res) * (float) res;
            p.y = round(v[1] / (float) res) * (float) res;
            p.z = round(v[2] / (float) res) * (float) res;
            arrow_voxels_pcl.points.push_back(p);
        }
    }
//    auto arrow_voxels_pcl = arrow_voxels.get_voxel_cloud();
    arrow_voxels_pcl.width = arrow_voxels_pcl.size();
    arrow_voxels_pcl.height = 1;
    arrow_voxels_pcl.is_dense = true;
    pcl::toROSMsg(arrow_voxels_pcl, arrow_voxels_msg);
    arrow_voxels_msg.header.frame_id = "map";
}

Eigen::Affine3f to_camera_pose(double axis_z_0, double axis_z_1, double axis_z_2,
                               double x_init, double y_init, double z_init) {
    Eigen::Matrix3f rotation = compute_rotation(axis_z_0, axis_z_1, axis_z_2);
    Eigen::Vector3f translation(x_init, y_init, z_init);
    Eigen::Affine3f camera_pose = Eigen::Affine3f::Identity();
    camera_pose.translate(translation);
    camera_pose.rotate(rotation);
    return camera_pose;
}

void fill_marked_img_pts(vector<marked_img_pt_type> &marked_img_pts, const Eigen::Affine3f &camera_pose) {
    observer->set_camera_pose(camera_pose);
    observer->render_to_marked_img_pts(marked_img_pts);
    sort(marked_img_pts.begin(), marked_img_pts.end());
}

vector<marked_diagonal_type> to_cone_arrows(const vector<marked_img_pt_pair_type> &intersection,
                                            const Eigen::Affine3f &camera_pose_1,
                                            const Eigen::Affine3f &camera_pose_2) {
    Eigen::Matrix3f r_1 = camera_pose_1.rotation().inverse();
    Eigen::Matrix3f r_2 = r_1 * camera_pose_2.rotation();
    Eigen::Vector3f t_2 = r_1 * (camera_pose_2.translation() - camera_pose_1.translation());

    vector<marked_diagonal_type> cone_arrows;
    for (auto p : intersection) {
        add_diagonal(p, camera_pose_1, r_2, t_2, cone_arrows);
    }
    return cone_arrows;
}

void fill_line_msgs(const vector<marked_img_pt_pair_type> &intersection,
                    const Eigen::Affine3f &camera_pose_1, const Eigen::Affine3f &camera_pose_2,
                    sensor_msgs::PointCloud2 &cones_inter_msg, visualization_msgs::Marker &line_list) {
    voxel_map_pcl line_voxels(res);

    for (auto p : intersection) {
        cone cone_1(get<1>(p), get<2>(p), camera_pose_1);
        cone cone_2(get<3>(p), get<4>(p), camera_pose_2);
        intersect_cones(cone_1, cone_2, line_voxels, line_list);
    }

    line_list.pose.orientation.z = 0.0;
    line_list.pose.orientation.y = 0.0;
    line_list.pose.orientation.z = 0.0;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    line_list.ns = "/lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.header.frame_id = "map";

    pcl::PointCloud<pcl::PointXYZ> cones_inter = line_voxels.get_voxel_cloud();
    cones_inter.width = cones_inter.size();
    cones_inter.height = 1;
    cones_inter.is_dense = true;
    pcl::toROSMsg(cones_inter, cones_inter_msg);
    cones_inter_msg.header.frame_id = "map";
}

void fill_observed_map_msg(const vector<marked_img_pt_pair_type> &intersection,
                           sensor_msgs::PointCloud2 &observed_map_msg) {
    pcl::PointCloud<pcl::PointXYZ> observed_map_pcl;
    for (auto p : intersection) {
        observed_map_pcl.points.emplace_back((*get<0>(p))[0], (*get<0>(p))[1], (*get<0>(p))[2]);
    }
    observed_map_pcl.width = observed_map_pcl.size();
    observed_map_pcl.height = 1;
    observed_map_pcl.is_dense = true;
    pcl::toROSMsg(observed_map_pcl, observed_map_msg);
    observed_map_msg.header.frame_id = "map";
}

struct cone_keeper {
    cone_keeper() {
        double pixel_cone_angle_2 = observer->get_pixel_cone_angle_2();
        l_dir << -pixel_cone_angle_2, 1;
        r_dir << +pixel_cone_angle_2, 1;
        double cos_angle = cos(pixel_cone_angle_2);
        double sin_angle = sin(pixel_cone_angle_2);
        pixel_norm_l << sin_angle, +cos_angle, -cos_angle, sin_angle;
        pixel_norm_r << sin_angle, -cos_angle, +cos_angle, sin_angle;
    }

    void update_marked_cones(const Eigen::Affine3f &camera_pose_1, const Eigen::Affine3f &camera_pose_2,
                             const vector<marked_img_pt_pair_type> &pts_inter) {
        for (const auto &pt : pts_inter) {
            auto marked_cone_it = marked_cones.find(get<0>(pt));
            marked_cone_value_type *cone_0;
            if (marked_cone_it == marked_cones.end()) {
                Eigen::Vector3f a = to_vec_3d(get<1>(pt), get<2>(pt), camera_pose_1.rotation()).normalized();
                cone_0 = &marked_cones.insert({get<0>(pt),
                                               {camera_pose_1.translation(), a, 0.01, 100}}).first->second;
            } else {
                cone_0 = &marked_cone_it->second;
            }

            Eigen::Vector3f C = get<0>(*cone_0);
            Eigen::Vector3f P = camera_pose_2.translation();
            Eigen::Vector3f a = get<1>(*cone_0);
            Eigen::Vector3f v = to_vec_3d(get<3>(pt), get<4>(pt), camera_pose_2.rotation()).normalized();
            Eigen::Vector3f r = a.cross(v).normalized();
            Eigen::Vector3f b = a.cross(r);
            Eigen::Matrix<float, 2, 3> camera_basis_2d;
            camera_basis_2d << b[0], b[1], b[2], a[0], a[1], a[2];
            Eigen::Vector2d P_ = (camera_basis_2d * (P - C)).cast<double>();
            Eigen::Vector2d v_ = (camera_basis_2d * v * 100).cast<double>();

            double h_1 = get<2>(*cone_0);
            double h_2 = get<3>(*cone_0);
            list<Eigen::Vector2d> points;
            points.emplace_back(0, 0);
            points.emplace_back(r_dir * h_2);
            points.emplace_back(l_dir * h_2);

            Eigen::Vector2d p_n_l = pixel_norm_l * v_;
            Eigen::Vector2d p_n_r = pixel_norm_r * v_;
            intersect_points_with_plane(points, P_, p_n_l);
            intersect_points_with_plane(points, P_, p_n_r);

            v_.normalize();
            double min_c, max_c, min_p, max_p;
            min_c = max_c = (*points.begin())[1];
            min_p = max_p = (*points.begin() - P_).dot(v_);
            for (auto it = ++points.begin(); it != points.end(); ++it) {
                double d_c = (*it)[1];
                double d_p = (*it - P_).dot(v_);
                min_c = min(min_c, d_c);
                min_p = min(min_p, d_p);
                max_c = max(max_c, d_c);
                max_p = max(max_p, d_p);
            }
            min_c = max(min_c, h_1);
            double V_0 = h_2 * h_2 * (h_2 - h_1);
            double V_c = max_c * max_c * (max_c - min_c);
            double V_p = max_p * max_p * (max_p - min_p);
            if (V_p <= V_c && V_p <= V_0) {
                get<0>(*cone_0) = camera_pose_2.translation();
                get<1>(*cone_0) = v;
                get<2>(*cone_0) = min_p;
                get<3>(*cone_0) = max_p;
            } else if (V_c <= V_0) {
                get<2>(*cone_0) = min_c;
                get<3>(*cone_0) = max_c;
            }
        }
    }

    Eigen::Vector2d l_dir;
    Eigen::Vector2d r_dir;
    Eigen::Matrix2d pixel_norm_l;
    Eigen::Matrix2d pixel_norm_r;

    map<const Eigen::Vector3d *, marked_cone_value_type> marked_cones;
};

void fill_keeper_arrows_msg(const cone_keeper &cone_keeper_, visualization_msgs::MarkerArray &keeper_arrows_msg,
                            double r, double g, double b, double w) {
    for (auto it : cone_keeper_.marked_cones) {
        Eigen::Vector3f v1 = get<0>(it.second) + get<1>(it.second) * get<2>(it.second);
        Eigen::Vector3f v2 = get<0>(it.second) + get<1>(it.second) * get<3>(it.second);
        append_marker_array_msg(v1, v2, r, g, b, w, w, false, keeper_arrows_msg);
    }
}

void prepare_msgs(sensor_msgs::PointCloud2 &observed_map_msg, sensor_msgs::PointCloud2 &cones_inter_msg,
                  sensor_msgs::PointCloud2 &arrow_voxels_msg, visualization_msgs::Marker &line_list,
                  visualization_msgs::MarkerArray &cone_arrows_msg, visualization_msgs::MarkerArray &inter_arrows_msg,
                  visualization_msgs::MarkerArray &keeper_arrows_12_msg,
                  visualization_msgs::MarkerArray &keeper_arrows_23_msg,
                  visualization_msgs::MarkerArray &z_arrows) {
    Eigen::Affine3f camera_pose_1 = to_camera_pose(axis_z1_0, axis_z1_1, axis_z1_2, x1_init, y1_init, z1_init);
    Eigen::Affine3f camera_pose_2 = to_camera_pose(axis_z2_0, axis_z2_1, axis_z2_2, x2_init, y2_init, z2_init);
    Eigen::Affine3f camera_pose_3 = to_camera_pose(axis_z3_0, axis_z3_1, axis_z3_2, x3_init, y3_init, z3_init);

    auto t00 = ros::Time::now();
    vector<marked_img_pt_type> marked_img_pts_1, marked_img_pts_2, marked_img_pts_3;
    fill_marked_img_pts(marked_img_pts_1, camera_pose_1);
    fill_marked_img_pts(marked_img_pts_2, camera_pose_2);
    fill_marked_img_pts(marked_img_pts_3, camera_pose_3);

    auto t01 = ros::Time::now();
    vector<marked_img_pt_pair_type> pts_inter_12 = intersect_sorted_marked_img_pts(marked_img_pts_1, marked_img_pts_2);
    vector<marked_img_pt_pair_type> pts_inter_23 = intersect_sorted_marked_img_pts(marked_img_pts_2, marked_img_pts_3);

    auto t02 = ros::Time::now();
    vector<marked_diagonal_type> cone_arrows = to_cone_arrows(pts_inter_12, camera_pose_1, camera_pose_2);
    auto t03 = ros::Time::now();
    fill_line_msgs(pts_inter_12, camera_pose_1, camera_pose_2, cones_inter_msg, line_list);

    auto t04 = ros::Time::now();
    fill_cone_arrows_msg(cone_arrows, cone_arrows_msg, 0, 0, 1, 0.06);
    auto t05 = ros::Time::now();
    shade_diagonals(cone_arrows, arrow_voxels_msg);
    auto t06 = ros::Time::now();
    intersect_diagonals(cone_arrows, marked_img_pts_3, camera_pose_3);
    auto t07 = ros::Time::now();
    fill_cone_arrows_msg(cone_arrows, inter_arrows_msg, 0, 1, 0, 0.1);

    cone_keeper cone_keeper_123;
    auto t08 = ros::Time::now();
    cone_keeper_123.update_marked_cones(camera_pose_1, camera_pose_2, pts_inter_12);
    auto t09 = ros::Time::now();
    fill_keeper_arrows_msg(cone_keeper_123, keeper_arrows_12_msg, 0, 1, 1, 0.06);
    auto t10 = ros::Time::now();
    cone_keeper_123.update_marked_cones(camera_pose_2, camera_pose_3, pts_inter_23);
    auto t11 = ros::Time::now();
    fill_keeper_arrows_msg(cone_keeper_123, keeper_arrows_23_msg, 1, 0, 1, 0.08);
    auto t12 = ros::Time::now();

    ROS_WARN("t_010 = %f", (t01 - t00).toSec() / 3);
    ROS_WARN("t_021 = %f", (t02 - t01).toSec() / 2);
    ROS_WARN("t_032 = %f", (t03 - t02).toSec());
    ROS_WARN("t_043 = %f", (t04 - t03).toSec());
    ROS_WARN("t_054 = %f", (t05 - t04).toSec());
    ROS_WARN("t_065 = %f", (t06 - t05).toSec());
    ROS_WARN("t_076 = %f", (t07 - t06).toSec());
    ROS_WARN("t_087 = %f", (t08 - t07).toSec());
    ROS_WARN("t_098 = %f", (t09 - t08).toSec());
    ROS_WARN("t_109 = %f", (t10 - t09).toSec());
    ROS_WARN("t_110 = %f", (t11 - t10).toSec());
    ROS_WARN("t_121 = %f", (t12 - t11).toSec());

    fill_observed_map_msg(pts_inter_12, observed_map_msg);

    add_arrow_marker(camera_pose_1, z_arrows);
    add_arrow_marker(camera_pose_2, z_arrows);
    add_arrow_marker(camera_pose_3, z_arrows);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map exploring test");
    ros::NodeHandle node_handle("~");

    ros::Publisher all_map_pub         = node_handle.advertise<sensor_msgs::PointCloud2>("all_map", 1);
    ros::Publisher marked_map_pub      = node_handle.advertise<sensor_msgs::PointCloud2>("marked_map", 1);
    ros::Publisher observed_map_pub    = node_handle.advertise<sensor_msgs::PointCloud2>("observed_map", 1);
    ros::Publisher cones_inter_pub     = node_handle.advertise<sensor_msgs::PointCloud2>("cones_inter", 1);
    ros::Publisher arrow_voxels_pub    = node_handle.advertise<sensor_msgs::PointCloud2>("arrow_voxels", 1);
    ros::Publisher inter_lines_pub     = node_handle.advertise<visualization_msgs::Marker>("inter_lines", 1);
    ros::Publisher cone_arrows_pub     = node_handle.advertise<visualization_msgs::MarkerArray>("cone_arrows", 1);
    ros::Publisher inter_arrows_pub    = node_handle.advertise<visualization_msgs::MarkerArray>("inter_arrows", 1);
    ros::Publisher keeper_arrows_1_pub = node_handle.advertise<visualization_msgs::MarkerArray>("keeper_arrows_1", 1);
    ros::Publisher keeper_arrows_2_pub = node_handle.advertise<visualization_msgs::MarkerArray>("keeper_arrows_2", 1);
    ros::Publisher z_arrows_pub        = node_handle.advertise<visualization_msgs::MarkerArray>("z_arrows", 1);

    double x_end, y_end, x_1, x_2, y_1, y_2, w_1, w_2, h_1, h_2, max_dist;
    int obs_num, seed, fov_hor;

    node_handle.param("map_boundary/lower_x", x_1,      -50.0);
    node_handle.param("map_boundary/upper_x", x_2,       50.0);
    node_handle.param("map_boundary/lower_y", y_1,      -50.0);
    node_handle.param("map_boundary/upper_y", y_2,       50.0);

    node_handle.param("obstacles/lower_w",    w_1,        0.6);
    node_handle.param("obstacles/upper_w",    w_2,        3.2);
    node_handle.param("obstacles/lower_h",    h_1,        1.0);
    node_handle.param("obstacles/upper_h",    h_2,       10.0);

    node_handle.param("map/resolution",       res,        0.1);
    node_handle.param("map/obstacles_num",    obs_num,    600);
    node_handle.param("map/seed",             seed,       1);

    node_handle.param("camera/width",         image_w,    1280);
    node_handle.param("camera/height",        image_h,    960);
    node_handle.param("camera/fov_hor",       fov_hor,    90);
    node_handle.param("camera/max_dist",      max_dist,  30.0);

    node_handle.param("copter/init_x",        x1_init,  -45.0);
    node_handle.param("copter/init_y",        y1_init,  -45.0);
    node_handle.param("copter/init_z",        z1_init,    2.0);
    node_handle.param("copter/end_x",         x_end,     30.0);
    node_handle.param("copter/end_y",         y_end,     30.0);
    node_handle.param("copter/axis_z_0",      axis_z1_0,  0.707);
    node_handle.param("copter/axis_z_1",      axis_z1_1,  0.707);
    node_handle.param("copter/axis_z_2",      axis_z1_2,  0.0);
    node_handle.param("copter/init_x2",       x2_init,  -44.5);
    node_handle.param("copter/init_y2",       y2_init,  -45.5);
    node_handle.param("copter/init_z2",       z2_init,    2.0);
    node_handle.param("copter/axis_z2_0",     axis_z2_0,  0.707);
    node_handle.param("copter/axis_z2_1",     axis_z2_1,  0.707);
    node_handle.param("copter/axis_z2_2",     axis_z2_2,  0.0);
    node_handle.param("copter/init_x3",       x3_init,  -44.0);
    node_handle.param("copter/init_y3",       y3_init,  -45.5);
    node_handle.param("copter/init_z3",       z3_init,    2.0);
    node_handle.param("copter/axis_z3_0",     axis_z3_0,  0.707);
    node_handle.param("copter/axis_z3_1",     axis_z3_1,  0.707);
    node_handle.param("copter/axis_z3_2",     axis_z3_2,  0.0);

//    ros::Rate rt(1);
//    for (int i = 0; i < 10 && ros::ok(); i++)
//        rt.sleep();

    double density = 0.1;
    marked_map_generator generator(x1_init, x_end, y1_init, y_end, x_1, x_2, y_1, y_2, h_1, h_2, w_1, w_2,
                                   res, density, obs_num, seed);
    generator.generate_map();

    auto global_map_pcl = generator.get_global_map_pcl();
    sensor_msgs::PointCloud2 global_map_msg;
    pcl::toROSMsg(global_map_pcl, global_map_msg);
    global_map_msg.header.frame_id = "map";

    auto marked_map_pcl = generator.get_marked_map_pcl();
    sensor_msgs::PointCloud2 marked_map_msg;
    pcl::toROSMsg(marked_map_pcl, marked_map_msg);
    marked_map_msg.header.frame_id = "map";

    auto shapes = generator.get_shapes();
    auto marked_points = generator.get_marked_points_vectors();
    observer = new marked_map_observer(marked_points, shapes, image_w, image_h, fov_hor, max_dist);
    focal_distance = observer->get_focal_distance();

    sensor_msgs::PointCloud2 observed_map_msg, cones_inter_msg, arrow_voxels_msg;
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray cone_arrows_msg, inter_arrows_msg,
            keeper_arrows_12_msg, keeper_arrows_23_msg, z_arrows;
    prepare_msgs(observed_map_msg, cones_inter_msg, arrow_voxels_msg, line_list,
                 cone_arrows_msg, inter_arrows_msg, keeper_arrows_12_msg, keeper_arrows_23_msg, z_arrows);

    ros::Rate loop_rate(1);
    for (int i = 0; i < 10 && ros::ok(); i++) {
        all_map_pub.publish(global_map_msg);
        marked_map_pub.publish(marked_map_msg);
        observed_map_pub.publish(observed_map_msg);
        cones_inter_pub.publish(cones_inter_msg);
        arrow_voxels_pub.publish(arrow_voxels_msg);
        inter_lines_pub.publish(line_list);
        cone_arrows_pub.publish(cone_arrows_msg);
        inter_arrows_pub.publish(inter_arrows_msg);
        keeper_arrows_1_pub.publish(keeper_arrows_12_msg);
        keeper_arrows_2_pub.publish(keeper_arrows_23_msg);
        z_arrows_pub.publish(z_arrows);
        loop_rate.sleep();
    }

    delete observer;
}