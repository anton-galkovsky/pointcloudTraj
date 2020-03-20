#include <ros/ros.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

static string trajs_archive_file, var_traj_length_file;
static pcl::PointCloud<pcl::PointXYZ> *cur_traj;
static double cur_traj_len;

void traj_com_pts_callback(const sensor_msgs::PointCloud2 &traj_msg) {
    pcl::PointCloud<pcl::PointXYZ> traj_pcl;
    pcl::fromROSMsg(traj_msg, traj_pcl);

    for (pcl::PointXYZ point : traj_pcl.points) {
        cur_traj->push_back(point);
    }

    ofstream fout(trajs_archive_file, ios::app);
    fout << traj_pcl.size() << "\n";
    for (pcl::PointXYZ point : traj_pcl.points) {
        fout << point.x << " " << point.y << " " << point.z << "\n";
    }
    fout.close();

    for (unsigned long i = 0; i < traj_pcl.size() - 1; i++) {
        cur_traj_len += pcl::euclideanDistance(traj_pcl[i], traj_pcl[i + 1]);
    }

    fout.open(var_traj_length_file, ios::out);
    fout << "TRAJ_LENGTH=" << cur_traj_len << "\n";
    fout.close();
}

void prepare_cur_traj_msg(sensor_msgs::PointCloud2 &cur_traj_msg) {
    cur_traj->width = cur_traj->size();
    pcl::toROSMsg(*cur_traj, cur_traj_msg);
    cur_traj_msg.header.frame_id = "map";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory keeper");
    ros::NodeHandle node_handle("~");

    ros::Subscriber traj_com_pts_sub =
            node_handle.subscribe("trajectory_commit_points", 50, traj_com_pts_callback);

    ros::Publisher cur_traj_pub =
            node_handle.advertise<sensor_msgs::PointCloud2>("current_trajectory", 1);

    node_handle.param("files/trajs_archive",
                      trajs_archive_file, string("/home/galanton/catkin_ws/trajectory_archive"));
    node_handle.param("files/var_traj_length",
                      var_traj_length_file, string("/home/galanton/catkin_ws/traj_length"));

    cur_traj_len = 0;
    cur_traj = new pcl::PointCloud<pcl::PointXYZ>();
    cur_traj->height = 1;
    cur_traj->is_dense = true;

    sensor_msgs::PointCloud2 cur_traj_msg;

    ros::Rate loop_rate(5);
    while (ros::ok()) {
        prepare_cur_traj_msg(cur_traj_msg);
        cur_traj_pub.publish(cur_traj_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete cur_traj;
}