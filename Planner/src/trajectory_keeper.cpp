#include <ros/ros.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <quadrotor_msgs/PositionCommand.h>

using namespace std;

static string trajs_archive_file, var_traj_length_file;
static pcl::PointCloud<pcl::PointXYZ> *cur_traj;
static double cur_traj_len;

void position_callback(const quadrotor_msgs::PositionCommand &cmd) {
    static pcl::PointXYZ last_pos(cmd.position.x, cmd.position.y, cmd.position.z);

    pcl::PointXYZ pos(cmd.position.x, cmd.position.y, cmd.position.z);
    cur_traj->push_back(pos);

    cur_traj_len += pcl::euclideanDistance(pos, last_pos);
    last_pos = pos;
}

void prepare_cur_traj_msg(sensor_msgs::PointCloud2 &cur_traj_msg) {
    cur_traj->width = cur_traj->size();
    pcl::toROSMsg(*cur_traj, cur_traj_msg);
    cur_traj_msg.header.frame_id = "map";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory keeper");
    ros::NodeHandle node_handle("~");

    ros::Subscriber position_sub = node_handle.subscribe("position", 100, position_callback);

    ros::Publisher cur_traj_pub = node_handle.advertise<sensor_msgs::PointCloud2>("current_trajectory", 1);

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

    ofstream fout(trajs_archive_file, ios::app);
    for (auto it : cur_traj->points) {
        fout << it.x << " " << it.y << " " << it.z << "\n";
    }
    fout.close();

    fout.open(var_traj_length_file, ios::out);
    fout << "TRAJ_LENGTH=" << cur_traj_len << "\n";
    fout.close();

    delete cur_traj;
}