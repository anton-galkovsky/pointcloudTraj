#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

void read_trajectories(const string &file, pcl::PointCloud<pcl::PointXYZ> &traj_pcl) {
    pcl::PointXYZ point;
    int n;

    ifstream fin(file, ios::in);
    while (fin.good() && !fin.eof()) {
        fin >> n;
        for (int i = 0; i < n; i++) {
            fin >> point.x >> point.y >> point.z;
            traj_pcl.points.push_back(point);
        }
    }
    fin.close();
}

void prepare_trajs_msg(const string &file, sensor_msgs::PointCloud2 &trajs_msg) {
    pcl::PointCloud<pcl::PointXYZ> traj_pcl;
    read_trajectories(file, traj_pcl);

    traj_pcl.width = traj_pcl.size();
    traj_pcl.height = 1;
    traj_pcl.is_dense = true;
    pcl::toROSMsg(traj_pcl, trajs_msg);
    trajs_msg.header.frame_id = "map";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectories comparison");
    ros::NodeHandle node_handle("~");

    ros::Publisher trajs_1_pub = node_handle.advertise<sensor_msgs::PointCloud2>("trajectories_1", 1);
    ros::Publisher trajs_2_pub = node_handle.advertise<sensor_msgs::PointCloud2>("trajectories_2", 1);

    string trajs_1_file, trajs_2_file;

    node_handle.param("files/trajs_1", trajs_1_file, string("/home/galanton/catkin_ws/trajectory_archive"));
    node_handle.param("files/trajs_2", trajs_2_file, string("/home/galanton/catkin_ws/trajectory_archive"));

    sensor_msgs::PointCloud2 trajs_1_msg, trajs_2_msg;

    prepare_trajs_msg(trajs_1_file, trajs_1_msg);
    prepare_trajs_msg(trajs_2_file, trajs_2_msg);

    ros::Rate loop_rate(1);
    for (int i = 0; i < 10 && ros::ok(); i++) {
        trajs_1_pub.publish(trajs_1_msg);
        trajs_2_pub.publish(trajs_2_msg);
        loop_rate.sleep();
    }
}