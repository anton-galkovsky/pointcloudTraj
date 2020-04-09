#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/MarkerArray.h>
#include "pointcloudTraj/map_observer.h"
#include "pointcloudTraj/cone_keeper.h"
#include "pointcloudTraj/utils.h"

using namespace std;

static int image_w, image_h, fov_hor;
static double x_init, y_init, z_init, x_end, y_end, res, max_dist, known_rad;
static bool was_pos_msg, was_map_mesh_msg, was_marked_points_msg, was_global_map_msg;

static pcl::PointCloud<pcl::PointXYZ> *known_map_pcl;

static vector<vector<Eigen::Vector3d>> marked_points;
static vector<vector<Eigen::Vector3d>> map_shapes;
static Eigen::Affine3f *camera_pose;

void prepare_map_scan(sensor_msgs::PointCloud2 &observed_map_msg, sensor_msgs::Image &image_msg,
                      visualization_msgs::MarkerArray &keeper_arrows_msg) {

    static marked_map_observer observer(marked_points, map_shapes, image_w, image_h, fov_hor, max_dist);
    static double pixel_cone_angle_2 = observer.get_pixel_cone_angle_2();
    static cone_keeper cone_keeper(pixel_cone_angle_2, observer.get_focal_distance(), image_w, image_h);

    observer.set_camera_pose(*camera_pose);

    auto marked_img_pts = observer.render_to_marked_img_pts();
    cone_keeper.add_marked_img_pts(*camera_pose, move(marked_img_pts));

    pcl::PointCloud<pcl::PointXYZ> observed_map_pcl(*known_map_pcl);
    auto marked_cones = cone_keeper.get_marked_cones();
    keeper_arrows_msg.markers.clear();
    int id = 0;
    for (auto it : marked_cones) {
        double h_1 = get<2>(it.second);
        double h_2 = get<3>(it.second);
        double l_1 = h_1 * pixel_cone_angle_2;
        double l_2 = h_2 * pixel_cone_angle_2;
        Eigen::Vector3f v1 = get<0>(it.second) + get<1>(it.second) * h_1;
        Eigen::Vector3f v2 = get<0>(it.second) + get<1>(it.second) * h_2;
        append_marker_array_msg(v1, v2, 0, 1, 1, l_1, l_2, false, keeper_arrows_msg, id++);

        observed_map_pcl.points.emplace_back(v1[0], v1[1], v1[2]);
    }

    observed_map_pcl.width = observed_map_pcl.size();
    observed_map_pcl.height = 1;
    observed_map_pcl.is_dense = true;
    pcl::toROSMsg(observed_map_pcl, observed_map_msg);
    observed_map_msg.header.frame_id = "map";

    auto depth_image = observer.render_to_img();
    unsigned char *rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(depth_image, image_w, image_h);

    image_msg.header.stamp = ros::Time::now();
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

void fill_points(bool &was_polygon_mesh_msg, vector<vector<Eigen::Vector3d>> &polygon_mesh,
                 const pcl_msgs::PolygonMesh &polygon_mesh_msg) {
    if (was_polygon_mesh_msg) {
        return;
    }
    was_polygon_mesh_msg = true;

    pcl::PointCloud<pcl::PointXYZ> map_mesh_pcl;
    pcl::fromROSMsg(polygon_mesh_msg.cloud, map_mesh_pcl);

    for (const auto& polygon_indexes : polygon_mesh_msg.polygons) {
        vector<Eigen::Vector3d> shape;
        for (int index : polygon_indexes.vertices) {
            pcl::PointXYZ &point = map_mesh_pcl[index];
            shape.emplace_back(point.x, point.y, point.z);
        }
        polygon_mesh.push_back(shape);
    }
}

void map_mesh_callback(const pcl_msgs::PolygonMesh &polygon_mesh_msg) {
    fill_points(was_map_mesh_msg, map_shapes, polygon_mesh_msg);
}

void marked_points_callback(const pcl_msgs::PolygonMesh &polygon_mesh_msg) {
    fill_points(was_marked_points_msg, marked_points, polygon_mesh_msg);
}

void position_callback(const quadrotor_msgs::PositionCommand &cmd) {
    was_pos_msg = true;

    static Eigen::Vector3f default_camera_axis_z =
            Eigen::Vector3f(x_end - x_init, y_end - y_init, 0).normalized();

    Eigen::Vector3f camera_axis_z(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
    if (camera_axis_z.norm() < 0.001) {
        camera_axis_z = default_camera_axis_z;
    } else {
        camera_axis_z /= camera_axis_z.norm();
    }
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
    kd_tree.radiusSearch(point, known_rad, points_indexes, points_distances);

    known_map_pcl = new pcl::PointCloud<pcl::PointXYZ>(global_map_pcl, points_indexes);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera sensor");
    ros::NodeHandle node_handle("~");

    ros::Subscriber position_sub      = node_handle.subscribe("position",      1, position_callback);
    ros::Subscriber all_map_pub       = node_handle.subscribe("all_map",       1, all_map_callback);
    ros::Subscriber map_mesh_sub      = node_handle.subscribe("map_mesh",      1, map_mesh_callback);
    ros::Subscriber marked_points_sub = node_handle.subscribe("marked_points", 1, marked_points_callback);

    ros::Publisher observed_map_pub  = node_handle.advertise<sensor_msgs::PointCloud2>       ("observed_map",       1);
    ros::Publisher image_pub         = node_handle.advertise<sensor_msgs::Image>             ("observed_map_image", 1);
    ros::Publisher keeper_arrows_pub = node_handle.advertise<visualization_msgs::MarkerArray>("keeper_arrows",      1);

    double s_rate;

    node_handle.param("map/resolution",    res,        0.3);
    node_handle.param("map/known_radius",  known_rad, 10.0);

    node_handle.param("camera/sense_rate", s_rate,     3.0);
    node_handle.param("camera/width",      image_w,    320);
    node_handle.param("camera/height",     image_h,    240);
    node_handle.param("camera/fov_hor",    fov_hor,    90);
    node_handle.param("camera/max_dist",   max_dist,  30.0);

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

    camera_pose = new Eigen::Affine3f();

    sensor_msgs::PointCloud2 observed_map_msg;
    sensor_msgs::Image image_msg;
    visualization_msgs::MarkerArray keeper_arrows_msg;

    ros::Rate loop_rate(s_rate);
    while (ros::ok() && (!was_pos_msg || !was_map_mesh_msg || !was_marked_points_msg || !was_global_map_msg)) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    while (ros::ok()) {
        prepare_map_scan(observed_map_msg, image_msg, keeper_arrows_msg);
        observed_map_pub.publish(observed_map_msg);
        image_pub.publish(image_msg);
        keeper_arrows_pub.publish(keeper_arrows_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete camera_pose;
}