#include <ros/ros.h>
#include <fstream>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
#include <quadrotor_msgs/PositionCommand.h>

using namespace std;

static pcl::search::KdTree<pcl::PointXYZ> kd_tree;
static bool was_global_map_msg;
static double col_rad;
static string var_exit_code_file;

//  0    OK
//  1    Can't find a path, mission stall, please reset the target
//  2    Cannot find a feasible and optimal solution, somthing wrong with the mosek solver ...
//  3    Collision Occur, Stop
//  4    max_time_sec has passed
//  5    default
void print_code(int code) {
    ofstream fout(var_exit_code_file);
    fout << "EXIT_CODE=" << code << "\n";
    fout.close();
}

void status_code_callback(const std_msgs::Int8 &int_msg) {
    print_code(int_msg.data);
    ros::shutdown();
}

void position_callback(const quadrotor_msgs::PositionCommand &cmd) {
    if (!was_global_map_msg) {
        return;
    }

    pcl::PointXYZ point(cmd.position.x, cmd.position.y, cmd.position.z);
    vector<int> points_indexes;
    vector<float> points_distances;
    kd_tree.nearestKSearch(point, 1, points_indexes, points_distances);
    if (!points_distances.empty() && sqrt(points_distances[0]) < col_rad) {
        print_code(3);
        ros::shutdown();
    }
}

void all_map_callback(const sensor_msgs::PointCloud2 &global_map_msg) {
    if (was_global_map_msg) {
        return;
    }
    was_global_map_msg = true;

    pcl::PointCloud<pcl::PointXYZ> global_map_pcl;
    pcl::fromROSMsg(global_map_msg, global_map_pcl);
    kd_tree.setInputCloud(global_map_pcl.makeShared());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "status inspector");
    ros::NodeHandle node_handle("~");

    ros::Subscriber position_sub    = node_handle.subscribe("position",    1,  position_callback);
    ros::Subscriber all_map_pub     = node_handle.subscribe("all_map",     1,  all_map_callback);
    ros::Subscriber status_code_sub = node_handle.subscribe("status_code", 50, status_code_callback);

    int max_time_sec;

    node_handle.param("max_time_sec",        max_time_sec,       120);
    node_handle.param("traj/col_rad",        col_rad,            0.4);
    node_handle.param("files/var_exit_code", var_exit_code_file, string("/home/galanton/catkin_ws/exit_code"));

    print_code(5);

    was_global_map_msg = false;

    int rate = 5;
    ros::Rate loop_rate(rate);
    for (int i = 0; i < max_time_sec * rate && ros::ok(); i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (ros::ok()) {
        print_code(4);
    }
}