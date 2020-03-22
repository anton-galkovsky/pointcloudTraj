#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "pointcloudTraj/map_observer.h"
#include "pointcloudTraj/voxel_map.h"

using namespace std;

static int image_w, image_h, fov_hor;
static double x_init, x_end, y_init, y_end, res;
static bool was_pos_msg, was_map_msg;

static vector<vector<Eigen::Vector3d>> map_shapes;
static Eigen::Affine3f *camera_pose;

void prepare_map_scan(sensor_msgs::PointCloud2 &observed_map_msg, sensor_msgs::Image &image_msg) {
    if (!was_pos_msg || !was_map_msg) {
        return;
    }

    static auto observer = img_pcl_map_observer(map_shapes, image_w, image_h, fov_hor);
    static auto observed_voxel_map = voxel_map(res);

    observer.set_camera_pose(*camera_pose);

    auto scan_pcl = observer.render_to_pcl();
    observed_voxel_map.add_point_cloud(*scan_pcl);

    auto observed_map_pcl = observed_voxel_map.get_voxel_cloud();
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

void map_mesh_callback(const pcl_msgs::PolygonMesh &polygon_mesh_msg) {
    if (was_map_msg) {
        return;
    }
    was_map_msg = true;

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "optical sensor");
    ros::NodeHandle node_handle("~");

    ros::Subscriber map_mesh_pub = node_handle.subscribe("map_mesh", 1, map_mesh_callback);
    ros::Subscriber position_sub = node_handle.subscribe("position", 1, position_callback);

    ros::Publisher observed_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("observed_map", 1);
    ros::Publisher image_pub        = node_handle.advertise<sensor_msgs::Image>("observed_map_image", 1);

    double s_rate;

    node_handle.param("map/resolution",    res,     0.3);

    node_handle.param("camera/sense_rate", s_rate,  3.0);
    node_handle.param("camera/width",      image_w, 320);
    node_handle.param("camera/height",     image_h, 240);
    node_handle.param("camera/fov_hor",    fov_hor, 90);

    node_handle.param("copter/init_x",     x_init, -45.0);
    node_handle.param("copter/init_y",     y_init, -45.0);
    node_handle.param("copter/end_x",      x_end,   30.0);
    node_handle.param("copter/end_y",      y_end,   30.0);

    was_pos_msg = false;
    was_map_msg = false;

    camera_pose = new Eigen::Affine3f();

    auto observed_map_msg = sensor_msgs::PointCloud2();
    auto image_msg = sensor_msgs::Image();

    ros::Rate loop_rate(s_rate);
    while (ros::ok()) {
        prepare_map_scan(observed_map_msg, image_msg);
        observed_map_pub.publish(observed_map_msg);
        image_pub.publish(image_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete camera_pose;
}