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
static double axis_z1_0, axis_z1_1, axis_z1_2;
static double axis_z2_0, axis_z2_1, axis_z2_2;

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

void add_diagonal(int x_1, int y_1, int x_2, int y_2, const Eigen::Affine3f &pose_1,
                  const Eigen::Matrix3f &rotation_2, const Eigen::Vector3f &t_2,
                  visualization_msgs::MarkerArray &cone_arrows) {
//    Eigen::Matrix3f rotation_2 = pose_1.rotation().inverse() * pose_2.rotation();
//    Eigen::Vector3f translation_2 = pose_2.translation() - pose_1.translation();
//    Eigen::Vector3f t_2 = pose_1.rotation().inverse() * translation_2;

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

    static int id = 0;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "/cone_arrows";
    arrow.id = id++;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.06;
    arrow.scale.y = 0.06;
    arrow.scale.z = 0.01;
    arrow.color.b = 1;
    arrow.color.a = 1;

    geometry_msgs::Point p;
    p.x = p_min[0];
    p.y = p_min[1];
    p.z = p_min[2];
    arrow.points.push_back(p);
    p.x = p_max[0];
    p.y = p_max[1];
    p.z = p_max[2];
    arrow.points.push_back(p);
    cone_arrows.markers.push_back(arrow);
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

void intersect_points_with_plane(list<Eigen::Vector3f> &points,
                                 const Eigen::Vector3f &plane_point, const Eigen::Vector3f &normal) {
    for (auto it_1 = points.begin(), it = it_1; it != points.end();) {
        ++it_1;
        if (it_1 == points.end()) {
            it_1 = points.begin();
        }
        double n_dot_i = (*it - plane_point).dot(normal);
        double n_dot_i_1 = (*it_1 - plane_point).dot(normal);
//        double n_i = (*it).dot(normal);
//        double n_i_1 = (*it_1).dot(normal);
//        double n_p = (plane_point).dot(normal);
        if ((n_dot_i < -0.01 && n_dot_i_1 > 0.01) || (n_dot_i_1 < -0.01 && n_dot_i > 0.01)) {
//        if ((n_i < n_p && n_i_1 > n_p) || (n_i_1 < n_p && n_i > n_p)) {
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
//        double n_i = (*it).dot(normal);
//        double n_p = (plane_point).dot(normal);
        if ((*it - plane_point).dot(normal) < -0.01) {
//        if (n_i < n_p) {
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

vector<tuple<const Eigen::Vector3d *, int, int, int, int>> intersect_sorted_marked_img_pts(
        const vector<tuple<const Eigen::Vector3d *, int, int>> &marked_img_pts_1,
        const vector<tuple<const Eigen::Vector3d *, int, int>> &marked_img_pts_2) {
    vector<tuple<const Eigen::Vector3d *, int, int, int, int>> intersection;
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

void to_arrow_marker(const Eigen::Affine3f &camera_pose, visualization_msgs::Marker &arrow) {
    geometry_msgs::Point p;
    p.x = camera_pose.translation()[0];
    p.y = camera_pose.translation()[1];
    p.z = camera_pose.translation()[2];
    arrow.points.push_back(p);
    auto vec_z = camera_pose.translation() + camera_pose.rotation().col(2);
    p.x = vec_z[0];
    p.y = vec_z[1];
    p.z = vec_z[2];
    arrow.points.push_back(p);
}

void prepare_msgs(sensor_msgs::PointCloud2 &observed_map_msg, sensor_msgs::PointCloud2 &cones_inter_msg,
                  visualization_msgs::Marker &line_list, visualization_msgs::MarkerArray &cone_arrows,
                  visualization_msgs::MarkerArray &z_arrows) {
    Eigen::Matrix3f rotation_1 = compute_rotation(axis_z1_0, axis_z1_1, axis_z1_2);
    Eigen::Matrix3f rotation_2 = compute_rotation(axis_z2_0, axis_z2_1, axis_z2_2);
    Eigen::Vector3f translation_1(x1_init, y1_init, z1_init);
    Eigen::Vector3f translation_2(x2_init, y2_init, z2_init);
    Eigen::Affine3f camera_pose_1 = Eigen::Affine3f::Identity();
    Eigen::Affine3f camera_pose_2 = Eigen::Affine3f::Identity();
    camera_pose_1.translate(translation_1);
    camera_pose_2.translate(translation_2);
    camera_pose_1.rotate(rotation_1);
    camera_pose_2.rotate(rotation_2);

    observer->set_camera_pose(camera_pose_1);
    vector<tuple<const Eigen::Vector3d *, int, int>> marked_img_pts_1;
    observer->render_to_marked_img_pts(marked_img_pts_1);

    observer->set_camera_pose(camera_pose_2);
    vector<tuple<const Eigen::Vector3d *, int, int>> marked_img_pts_2;
    observer->render_to_marked_img_pts(marked_img_pts_2);

    sort(marked_img_pts_1.begin(), marked_img_pts_1.end());
    sort(marked_img_pts_2.begin(), marked_img_pts_2.end());

    vector<tuple<const Eigen::Vector3d *, int, int, int, int>> intersection =
            intersect_sorted_marked_img_pts(marked_img_pts_1, marked_img_pts_2);

    voxel_map_pcl line_voxels(res);

    Eigen::Matrix3f r_1 = camera_pose_1.rotation().inverse();
    Eigen::Matrix3f r_2 = r_1 * camera_pose_2.rotation();
    Eigen::Vector3f t_2 = r_1 * (camera_pose_2.translation() - camera_pose_1.translation());

    for (auto p : intersection) {
        add_diagonal(get<1>(p), get<2>(p), get<3>(p), get<4>(p),
                     camera_pose_1, r_2, t_2, cone_arrows);

        cone cone_1(get<1>(p), get<2>(p), camera_pose_1);
        cone cone_2(get<3>(p), get<4>(p), camera_pose_2);
        intersect_cones(cone_1, cone_2, line_voxels, line_list);
    }

    pcl::PointCloud<pcl::PointXYZ> cones_inter = line_voxels.get_voxel_cloud();
    cones_inter.width = cones_inter.size();
    cones_inter.height = 1;
    cones_inter.is_dense = true;
    pcl::toROSMsg(cones_inter, cones_inter_msg);
    cones_inter_msg.header.frame_id = "map";


    pcl::PointCloud<pcl::PointXYZ> observed_map_pcl;
    for (auto p : intersection) {
        observed_map_pcl.points.emplace_back((*get<0>(p))[0], (*get<0>(p))[1], (*get<0>(p))[2]);
    }
    observed_map_pcl.width = observed_map_pcl.size();
    observed_map_pcl.height = 1;
    observed_map_pcl.is_dense = true;
    pcl::toROSMsg(observed_map_pcl, observed_map_msg);
    observed_map_msg.header.frame_id = "map";


    line_list.header.frame_id = "map";
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

    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "/z_arrows";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.06;
    arrow.scale.y = 0.1;
    arrow.color.g = 1;
    arrow.color.a = 1;

    arrow.id = 0;
    to_arrow_marker(camera_pose_1, arrow);
    z_arrows.markers.push_back(arrow);

    arrow.points.clear();
    arrow.id = 1;
    to_arrow_marker(camera_pose_2, arrow);
    z_arrows.markers.push_back(arrow);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map exploring test");
    ros::NodeHandle node_handle("~");

    ros::Publisher all_map_pub      = node_handle.advertise<sensor_msgs::PointCloud2>("all_map", 1);
    ros::Publisher marked_map_pub   = node_handle.advertise<sensor_msgs::PointCloud2>("marked_map", 1);
    ros::Publisher observed_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("observed_map", 1);
    ros::Publisher cones_inter_pub  = node_handle.advertise<sensor_msgs::PointCloud2>("cones_inter", 1);
    ros::Publisher inter_lines_pub  = node_handle.advertise<visualization_msgs::Marker>("inter_lines", 1);
    ros::Publisher cone_arrows_pub  = node_handle.advertise<visualization_msgs::MarkerArray>("cone_arrows", 1);
    ros::Publisher z_arrows_pub     = node_handle.advertise<visualization_msgs::MarkerArray>("z_arrows", 1);

    double x_end, y_end, x_1, x_2, y_1, y_2, w_1, w_2, h_1, h_2;
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
    observer = new marked_map_observer(marked_points, shapes, image_w, image_h, fov_hor);
    focal_distance = observer->get_focal_distance();

    sensor_msgs::PointCloud2 observed_map_msg;
    sensor_msgs::PointCloud2 cones_inter_msg;
    visualization_msgs::Marker line_list;
    visualization_msgs::MarkerArray cone_arrows;
    visualization_msgs::MarkerArray z_arrows;
    prepare_msgs(observed_map_msg, cones_inter_msg, line_list, cone_arrows, z_arrows);

    ros::Rate loop_rate(1);
    for (int i = 0; i < 10 && ros::ok(); i++) {
        all_map_pub.publish(global_map_msg);
        marked_map_pub.publish(marked_map_msg);
        observed_map_pub.publish(observed_map_msg);
        cones_inter_pub.publish(cones_inter_msg);
        inter_lines_pub.publish(line_list);
        cone_arrows_pub.publish(cone_arrows);
        z_arrows_pub.publish(z_arrows);
        loop_rate.sleep();
    }

    delete observer;
}