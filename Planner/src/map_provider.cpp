#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloudTraj/map_generator.h"

using namespace std;

void prepare_map_mesh_msg(const vector<vector<Eigen::Vector3d>> *shapes, pcl_msgs::PolygonMesh &polygon_mesh_msg) {
    pcl::PointCloud<pcl::PointXYZ> map_mesh_pcl;
    int k = 0;
    for (const auto &shape : *shapes) {
        pcl_msgs::Vertices polygon_indexes;
        for (auto point : shape) {
            map_mesh_pcl.points.emplace_back(point[0], point[1], point[2]);
            polygon_indexes.vertices.push_back(k);
            k++;
        }
        polygon_mesh_msg.polygons.push_back(polygon_indexes);
    }
    sensor_msgs::PointCloud2 map_mesh_msg;
    pcl::toROSMsg(map_mesh_pcl, map_mesh_msg);
    polygon_mesh_msg.cloud = map_mesh_msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map provider");
    ros::NodeHandle node_handle("~");

    ros::Publisher all_map_pub  = node_handle.advertise<sensor_msgs::PointCloud2>("all_map", 1);
    ros::Publisher map_mesh_pub = node_handle.advertise<pcl_msgs::PolygonMesh>("map_mesh", 1);

    double x_init, x_end, y_init, y_end, x_1, x_2, y_1, y_2, h_1, h_2, w_1, w_2, res;
    int obs_num, seed;

    node_handle.param("map_boundary/lower_x", x_1,    -50.0);
    node_handle.param("map_boundary/upper_x", x_2,     50.0);
    node_handle.param("map_boundary/lower_y", y_1,    -50.0);
    node_handle.param("map_boundary/upper_y", y_2,     50.0);

    node_handle.param("obstacles/lower_w",    w_1,     0.6);
    node_handle.param("obstacles/upper_w",    w_2,     3.2);
    node_handle.param("obstacles/lower_h",    h_1,     1.0);
    node_handle.param("obstacles/upper_h",    h_2,     10.0);

    node_handle.param("map/resolution",       res,     0.3);
    node_handle.param("map/obstacles_num",    obs_num, 600);
    node_handle.param("map/seed",             seed,    1);

    node_handle.param("copter/init_x",        x_init, -45.0);
    node_handle.param("copter/init_y",        y_init, -45.0);
    node_handle.param("copter/end_x",         x_end,   30.0);
    node_handle.param("copter/end_y",         y_end,   30.0);

    map_generator generator(x_init, x_end, y_init, y_end, x_1, x_2, y_1, y_2, h_1, h_2, w_1, w_2, res, obs_num, seed);
    generator.generate_map();
    ROS_WARN("[Random map generator] Finished generating map");

    auto global_map_pcl = generator.get_global_map_pcl();
    sensor_msgs::PointCloud2 global_map_msg;
    pcl::toROSMsg(*global_map_pcl, global_map_msg);
    global_map_msg.header.frame_id = "map";

    auto shapes = generator.get_shapes();
    pcl_msgs::PolygonMesh polygon_mesh_msg;
    prepare_map_mesh_msg(shapes, polygon_mesh_msg);

    ros::Rate loop_rate(5);
    for (int i = 0; i < 25 && ros::ok(); i++) {
        all_map_pub.publish(global_map_msg);
        map_mesh_pub.publish(polygon_mesh_msg);
        loop_rate.sleep();
    }
}