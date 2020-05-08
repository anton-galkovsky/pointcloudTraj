#include <ros/ros.h>
#include <random>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/MarkerArray.h>
#include "pointcloudTraj/map_observer.h"
#include "pointcloudTraj/cone_keeper.h"
#include "pointcloudTraj/safety_controller.h"
#include "pointcloudTraj/utils.h"

using namespace std;

static int image_w, image_h, fov_hor, sf_img_w, sf_img_h;
static double x_init, y_init, z_init, x_end, y_end, res, max_dist, empty_rad, safe_rad;
static bool was_pos_msg, was_map_mesh_msg, was_marked_points_msg, was_global_map_msg;

static pcl::PointCloud<pcl::PointXYZ> *known_map_pcl;

static vector<vector<int>> marked_point_indexes_arr;
static vector<Eigen::Vector3d> marked_points_vec;
static vector<vector<Eigen::Vector3d>> map_shapes;
static Eigen::Affine3f *camera_pose;
static safety_controller *safety_ctrlr;

static bool valid_shift, valid_twirl;

void prepare_safe_sphere_msg(visualization_msgs::Marker &safe_sphere_msg, double safe_radius,
                             const Eigen::Vector3f &pos) {
    safe_sphere_msg.header.frame_id = "map";
    safe_sphere_msg.ns = "/sphere";
    safe_sphere_msg.type = visualization_msgs::Marker::SPHERE;
    safe_sphere_msg.action = visualization_msgs::Marker::ADD;
    safe_sphere_msg.pose.orientation.x = 0.0;
    safe_sphere_msg.pose.orientation.y = 0.0;
    safe_sphere_msg.pose.orientation.z = 0.0;
    safe_sphere_msg.pose.orientation.w = 1.0;
    safe_sphere_msg.color.a = 0.4;
    safe_sphere_msg.color.r = 0.0;
    safe_sphere_msg.color.g = 1.0;
    safe_sphere_msg.color.b = 1.0;
    safe_sphere_msg.id = 0;
    safe_sphere_msg.pose.position.x = pos[0];
    safe_sphere_msg.pose.position.y = pos[1];
    safe_sphere_msg.pose.position.z = pos[2];
    safe_sphere_msg.scale.x = 2 * safe_radius;
    safe_sphere_msg.scale.y = 2 * safe_radius;
    safe_sphere_msg.scale.z = 2 * safe_radius;
}

void prepare_observed_map_msg(sensor_msgs::PointCloud2 &observed_map_msg,
                              pcl::PointCloud<pcl::PointXYZ> &observed_map_pcl) {
    observed_map_pcl.width = observed_map_pcl.size();
    observed_map_pcl.height = 1;
    observed_map_pcl.is_dense = true;
    pcl::toROSMsg(observed_map_pcl, observed_map_msg);
    observed_map_msg.header.frame_id = "map";
}

void prepare_image_msg(sensor_msgs::Image &image_msg,
                       const float *depth_image) {
    unsigned char *rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(depth_image, image_w, image_h);

    image_msg.height = image_h;
    image_msg.width = image_w;
    image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    image_msg.step = image_w * 3;
    image_msg.data.resize(image_w * image_h * 3);
    for (int i = 0; i < image_w * image_h * 3; i++) {
        image_msg.data[i] = rgb_image[i];
    }
    delete rgb_image;
}

void prepare_traj_image_msg(sensor_msgs::Image &traj_image_msg,
                            tuple<const float *, const points_pixels_type, const points_pixels_type> image_info) {
    unsigned char *rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(
            get<0>(image_info), sf_img_w, sf_img_h, 0, INFINITY, true);
    traj_image_msg.width = sf_img_w;
    traj_image_msg.height = sf_img_h;
    traj_image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    traj_image_msg.step = sf_img_w * 3;
    traj_image_msg.data.resize(sf_img_w * sf_img_h * 3);
    for (int i = 0; i < sf_img_w * sf_img_h * 3; i++) {
        traj_image_msg.data[i] = rgb_image[i];
    }
    delete rgb_image;

    for (auto p : get<1>(image_info)) {
        int idx = p.second * sf_img_w + p.first;
        traj_image_msg.data[idx * 3 + 0] = 0;
        traj_image_msg.data[idx * 3 + 1] = 255;
        traj_image_msg.data[idx * 3 + 2] = 0;
    }

    for (auto p : get<2>(image_info)) {
        int idx = p.second * sf_img_w + p.first;
        traj_image_msg.data[idx * 3 + 0] = 255;
        traj_image_msg.data[idx * 3 + 1] = 0;
        traj_image_msg.data[idx * 3 + 2] = 0;
    }
}

void prepare_traj_points_msg(sensor_msgs::PointCloud2 &traj_points_msg, const std::list<Eigen::Vector3d> &points) {
    pcl::PointCloud<pcl::PointXYZ> traj_points_pcl;
    for (const auto &p : points) {
        traj_points_pcl.points.emplace_back(p[0], p[1], p[2]);
    }

    traj_points_pcl.width = traj_points_pcl.size();
    traj_points_pcl.height = 1;
    traj_points_pcl.is_dense = true;
    pcl::toROSMsg(traj_points_pcl, traj_points_msg);
    traj_points_msg.header.frame_id = "map";
}

void prepare_map_scan(sensor_msgs::PointCloud2 &observed_map_msg,
                      sensor_msgs::Image &image_msg, sensor_msgs::Image &traj_image_msg,
                      visualization_msgs::MarkerArray &keeper_arrows_msg, visualization_msgs::Marker &safe_sphere_msg,
                      geometry_msgs::Vector3 &shift_vec_msg, std_msgs::Float64 &twirl_yaw_msg,
                      sensor_msgs::PointCloud2 &safe_points_msg, sensor_msgs::PointCloud2 &unsafe_points_msg) {
    static double dist_stat_cur[200]{};
    int dist_stat_cur_num[200]{};
    double dist_stat_prev[200]{};
    for (int i = 0; i < 200; i++) {
        dist_stat_prev[i] = dist_stat_cur[i];
        dist_stat_cur[i] = 0;
    }

    static marked_map_observer observer(marked_point_indexes_arr, marked_points_vec, map_shapes,
                                        image_w, image_h, fov_hor, max_dist);
    static double pixel_cone_angle_2 = observer.get_pixel_cone_angle_2();
    static cone_keeper cone_keeper(empty_rad, Eigen::Vector3f(x_init, y_init, z_init),
                                   pixel_cone_angle_2, map_observer::get_focal_distance(fov_hor), image_w, image_h);
    static map<const Eigen::Vector3d *, int> observed_pts_filter;

    observer.set_camera_pose(*camera_pose);
    auto transl = camera_pose->translation();
    auto axes_x = camera_pose->rotation().col(0);
    auto axes_y = camera_pose->rotation().col(1);
    auto axes_z = camera_pose->rotation().col(2);

    auto marked_img_pts = observer.render_to_marked_img_pts();
    cone_keeper.add_marked_img_pts(*camera_pose, move(marked_img_pts));

    pcl::PointCloud<pcl::PointXYZ> observed_map_pcl(*known_map_pcl);
    auto marked_cones = cone_keeper.get_marked_cones();
    keeper_arrows_msg.markers.clear();
    int id = 0;
    double shift_dirs[36]{};
    int shift_dirs_num[36]{};
    for (auto it : marked_cones) {
        double h_1 = get<2>(it.second);
        double h_2 = get<3>(it.second);
        double l_1 = h_1 * pixel_cone_angle_2;
        double l_2 = h_2 * pixel_cone_angle_2;
        Eigen::Vector3f c = get<0>(it.second);
        Eigen::Vector3f v = get<1>(it.second);
        Eigen::Vector3f v1 = c + v * h_1;
        Eigen::Vector3f v2 = c + v * h_2;
        append_marker_array_msg(v1, v2, 0, 1, 1, l_1, l_2, false, keeper_arrows_msg, id++);

        auto filter_it = observed_pts_filter.insert({it.first, 0}).first;
        filter_it->second++;
        if (filter_it->second >= 2) {
            double h_sim = 2 * h_1 * h_2 / (h_1 + h_2);
            Eigen::Vector3f sim_v = c + v * h_sim;
            observed_map_pcl.points.emplace_back(sim_v[0], sim_v[1], sim_v[2]);

            double pos_dist = (sim_v - transl).norm();
            int dist_index = (int) round(min(pos_dist, 19.9) / 0.1);
            dist_stat_cur[dist_index] += h_2 - h_1;
            dist_stat_cur_num[dist_index]++;

            double safe_dist = (v1 - transl).norm();
            if (safe_dist < safe_rad) {
                valid_shift = true;
            }


            Eigen::Vector3f shift_dir = (v1 - transl).normalized().cross(axes_z);
            double shift_dir_norm = shift_dir.norm();
            if (shift_dir_norm < 0.01) {
                continue;
            }
            double shift_dir_coef = pow(1 - shift_dir_norm, 2);
            double angle = atan2(-shift_dir.dot(axes_y), shift_dir.dot(axes_x));
            angle += M_PI;
            int dir_ind = max(min((int) (angle / (M_PI / 36)), 35), 0);
            shift_dirs[dir_ind] += shift_dir_coef * max(h_2 - h_1, 1.0) / safe_dist;
            shift_dirs_num[dir_ind]++;
        }
    }
    for (int i = 0; i < 36; i++) {
        if (shift_dirs_num[i] != 0) {
            shift_dirs[i] /= shift_dirs_num[i];
        }
    }

    ofstream fout("/home/galanton/catkin_ws/stat", ios::app);
    for (int i = 0; i < 200; i++) {
        fout.width(6);
        fout.precision(2);
        fout << dist_stat_prev[i] - dist_stat_cur[i] / dist_stat_cur_num[i];
    }
    fout << "\n";
    fout.close();

    double safe_radius = cone_keeper.get_safe_radius();
    prepare_safe_sphere_msg(safe_sphere_msg, safe_radius, transl);

    prepare_observed_map_msg(observed_map_msg, observed_map_pcl);

    auto depth_image = observer.render_to_img();
    prepare_image_msg(image_msg, depth_image);

    auto cur_depth_image = cone_keeper.get_cur_depth_image();
    auto new_image_info = safety_ctrlr->add_image(*camera_pose, cur_depth_image);
    auto traj_points = safety_ctrlr->get_groups();
    prepare_traj_points_msg(safe_points_msg, traj_points.first);
    prepare_traj_points_msg(unsafe_points_msg, traj_points.second);
    prepare_traj_image_msg(traj_image_msg, new_image_info);


    double dir_max = 0;
    int dir_max_ind = 0;
    for (int i = 0; i < 18; i++) {
        if (shift_dirs[i] + shift_dirs[i + 18] > dir_max) {
            dir_max = shift_dirs[i] + shift_dirs[i + 18];
            dir_max_ind = i;
        }
    }
//    cout << "dir_max ================================= " << dir_max << endl;
//    if (dir_max > 30) {
//        valid_shift = true;
//    }
    if (valid_shift) {
        double angle = dir_max_ind * M_PI / 36 - M_PI;
        Eigen::Vector3f v = axes_x * cos(angle) - axes_y * sin(angle);
        v *= min(dir_max / 30, 1.0) + 1.0;
        shift_vec_msg.x = v[0];
        shift_vec_msg.y = v[1];
        shift_vec_msg.z = v[2];
    }


    int safe_pts = traj_points.first.size();
    int unsafe_pts = traj_points.second.size();
    if (safe_pts > 0.9 * (safe_pts + unsafe_pts)) {
        valid_twirl = true;

        double left_pts = shift_dirs[0] + shift_dirs[1] + shift_dirs[2] +
                          shift_dirs[33] + shift_dirs[34] + shift_dirs[35];
        double right_pts = shift_dirs[15] + shift_dirs[16] + shift_dirs[17] +
                           shift_dirs[18] + shift_dirs[19] + shift_dirs[20];
        Eigen::Vector2f v = Eigen::Vector2f(axes_x[0], axes_x[1]).normalized();
        twirl_yaw_msg.data = v[0];
        if (left_pts > right_pts) {
            twirl_yaw_msg.data *= -1;
        }
    }
}

void nearest_traj_callback(const sensor_msgs::PointCloud2 &nearest_traj_msg) {
    pcl::PointCloud<pcl::PointXYZ> nearest_traj_pcl;
    pcl::fromROSMsg(nearest_traj_msg, nearest_traj_pcl);
    list<Eigen::Vector3d> new_points;
    for (const auto &p : nearest_traj_pcl) {
        new_points.emplace_back(p.x, p.y, p.z);
    }
    safety_ctrlr->reset_controlled_points(new_points);
}

void map_mesh_callback(const pcl_msgs::PolygonMesh &polygon_mesh_msg) {
    if (was_map_mesh_msg) {
        return;
    }
    was_map_mesh_msg = true;

    pcl::PointCloud<pcl::PointXYZ> map_mesh_pcl;
    pcl::fromROSMsg(polygon_mesh_msg.cloud, map_mesh_pcl);

    for (const auto& polygon_indexes : polygon_mesh_msg.polygons) {
        vector<Eigen::Vector3d> shape;
        for (int index : polygon_indexes.vertices) {
            pcl::PointXYZ &point = map_mesh_pcl[index];
            shape.emplace_back(point.x, point.y, point.z);
        }
        map_shapes.push_back(shape);
    }
}

void marked_points_callback(const pcl_msgs::PolygonMesh &polygon_mesh_msg) {
    if (was_marked_points_msg) {
        return;
    }
    was_marked_points_msg = true;

    pcl::PointCloud<pcl::PointXYZ> marked_points_pcl;
    pcl::fromROSMsg(polygon_mesh_msg.cloud, marked_points_pcl);

    for (const auto &point : marked_points_pcl) {
        marked_points_vec.emplace_back(point.x, point.y, point.z);
    }

    for (const auto& polygon_indexes : polygon_mesh_msg.polygons) {
        vector<int> point_indexes;
        for (int index : polygon_indexes.vertices) {
            point_indexes.push_back(index);
        }
        marked_point_indexes_arr.push_back(point_indexes);
    }
}

void position_callback(const quadrotor_msgs::PositionCommand &cmd) {
    was_pos_msg = true;

    Eigen::Vector3f camera_axis_z(cos(cmd.yaw), sin(cmd.yaw), 0);
    Eigen::Vector3f camera_axis_x = camera_axis_z.cross(Eigen::Vector3f(0, 0, 1));
    camera_axis_x /= camera_axis_x.norm();

    Eigen::Vector3f camera_axis_y = camera_axis_z.cross(camera_axis_x);
    if (camera_axis_y[2] > 0) {
        camera_axis_x *= -1;
        camera_axis_y *= -1;
    }

    Eigen::Vector3f translation(cmd.position.x, cmd.position.y, cmd.position.z);
    Eigen::Matrix3f rotation;
    rotation << camera_axis_x, camera_axis_y, camera_axis_z;

    *camera_pose = Eigen::Affine3f::Identity();
    camera_pose->translate(translation);
    camera_pose->rotate(rotation);
}

void all_map_callback(const sensor_msgs::PointCloud2 &global_map_msg) {
    if (was_global_map_msg) {
        return;
    }
    was_global_map_msg = true;

    pcl::PointCloud<pcl::PointXYZ> global_map_pcl;
    pcl::fromROSMsg(global_map_msg, global_map_pcl);

    pcl::search::KdTree<pcl::PointXYZ> kd_tree;
    vector<int> points_indexes;
    vector<float> points_distances;
    pcl::PointXYZ point;
    point.x = (float) x_init;
    point.y = (float) y_init;
    point.z = (float) z_init;
    kd_tree.setInputCloud(global_map_pcl.makeShared());
    kd_tree.radiusSearch(point, empty_rad, points_indexes, points_distances);

    known_map_pcl = new pcl::PointCloud<pcl::PointXYZ>(global_map_pcl, points_indexes);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera sensor");
    ros::NodeHandle node_handle("~");

    ros::Subscriber position_sub      = node_handle.subscribe("position",      1, position_callback);
    ros::Subscriber all_map_pub       = node_handle.subscribe("all_map",       1, all_map_callback);
    ros::Subscriber map_mesh_sub      = node_handle.subscribe("map_mesh",      1, map_mesh_callback);
    ros::Subscriber marked_points_sub = node_handle.subscribe("marked_points", 1, marked_points_callback);
    ros::Subscriber nearest_traj_sub  = node_handle.subscribe("nearest_traj",  1, nearest_traj_callback);

    ros::Publisher shift_cmd_pub     = node_handle.advertise<geometry_msgs::Vector3>         ("shift_cmd",          1);
    ros::Publisher twirl_cmd_pub     = node_handle.advertise<std_msgs::Float64>              ("twirl_cmd",          1);
    ros::Publisher observed_map_pub  = node_handle.advertise<sensor_msgs::PointCloud2>       ("observed_map",       1);
    ros::Publisher safe_points_pub   = node_handle.advertise<sensor_msgs::PointCloud2>       ("safe_points",        1);
    ros::Publisher unsafe_points_pub = node_handle.advertise<sensor_msgs::PointCloud2>       ("unsafe_points",      1);
    ros::Publisher traj_image_pub    = node_handle.advertise<sensor_msgs::Image>             ("traj_image",         1);
    ros::Publisher image_pub         = node_handle.advertise<sensor_msgs::Image>             ("observed_map_image", 1);
    ros::Publisher keeper_arrows_pub = node_handle.advertise<visualization_msgs::MarkerArray>("keeper_arrows",      1);
    ros::Publisher safe_sphere_pub   = node_handle.advertise<visualization_msgs::Marker>     ("safe_sphere",        1);

    double s_rate;
    int buf_size;

    node_handle.param("traj/safe_rad",     safe_rad,   1.0);

    node_handle.param("map/resolution",    res,        0.3);
    node_handle.param("map/empty_rad",     empty_rad,  1.0);

    node_handle.param("camera/sense_rate", s_rate,     3.0);
    node_handle.param("camera/width",      image_w,    320);
    node_handle.param("camera/height",     image_h,    240);
    node_handle.param("camera/fov_hor",    fov_hor,    90);
    node_handle.param("camera/max_dist",   max_dist,  30.0);

    node_handle.param("safety/img_width",  sf_img_w,   120);
    node_handle.param("safety/img_height", sf_img_h,   90);
    node_handle.param("safety/buf_size",   buf_size,   50);

    node_handle.param("copter/init_x",     x_init,   -45.0);
    node_handle.param("copter/init_y",     y_init,   -45.0);
    node_handle.param("copter/init_z",     z_init,     2.0);
    node_handle.param("copter/end_x",      x_end,     30.0);
    node_handle.param("copter/end_y",      y_end,     30.0);

//    ros::Rate rt(1);
//    for (int i = 0; i < 10 && ros::ok(); i++)
//        rt.sleep();

    was_pos_msg = false;
    was_map_mesh_msg = false;
    was_marked_points_msg = false;
    was_global_map_msg = false;

    valid_shift = false;
    valid_twirl = false;

    ofstream fout("/home/galanton/catkin_ws/stat");
    for (int i = 0; i < 200; i++) {
        fout.width(6);
        fout.precision(2);
        fout << i * 0.1;
    }
    fout << "\n\n";
    fout.close();

    camera_pose = new Eigen::Affine3f();
    Eigen::Vector3f translation(x_init, y_init, z_init);
    camera_pose->translate(translation);

    safety_ctrlr = new safety_controller(sf_img_w, sf_img_h, buf_size,
                                         map_observer::get_focal_distance(fov_hor), image_w, image_h);

    sensor_msgs::PointCloud2 observed_map_msg, safe_points_msg, unsafe_points_msg;
    sensor_msgs::Image image_msg, traj_image_msg;
    visualization_msgs::MarkerArray keeper_arrows_msg;
    visualization_msgs::Marker safe_sphere_msg;
    geometry_msgs::Vector3 shift_vec_msg;
    std_msgs::Float64 twirl_yaw_msg;

    ros::Rate loop_rate(s_rate);
    while (ros::ok() && (!was_pos_msg || !was_map_mesh_msg || !was_marked_points_msg || !was_global_map_msg)) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    while (ros::ok()) {
        prepare_map_scan(observed_map_msg, image_msg, traj_image_msg, keeper_arrows_msg, safe_sphere_msg,
                         shift_vec_msg, twirl_yaw_msg, safe_points_msg, unsafe_points_msg);
        observed_map_pub.publish(observed_map_msg);
        traj_image_pub.publish(traj_image_msg);
        image_pub.publish(image_msg);
        keeper_arrows_pub.publish(keeper_arrows_msg);
        safe_sphere_pub.publish(safe_sphere_msg);
        unsafe_points_pub.publish(unsafe_points_msg);
        safe_points_pub.publish(safe_points_msg);
        if (valid_shift) {
            shift_cmd_pub.publish(shift_vec_msg);
            valid_shift = false;
//            ROS_WARN("[Camera] Send shift cmd");
        }
        if (valid_twirl) {
            twirl_cmd_pub.publish(twirl_yaw_msg);
            valid_twirl = false;
//            ROS_WARN("[Camera] Send twirl cmd");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete camera_pose;
    delete safety_ctrlr;
}