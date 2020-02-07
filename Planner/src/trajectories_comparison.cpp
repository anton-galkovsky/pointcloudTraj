#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

void read_trajectories(const std::string &file, pcl::PointCloud<pcl::PointXYZ> &traj_pcl) {
    pcl::PointXYZ point;
    int n;

    std::ifstream fin(file, std::ios::in);
    while (fin.good() && !fin.eof()) {
        fin >> n;
        for (int i = 0; i < n; i++) {
            fin >> point.x >> point.y >> point.z;
            traj_pcl.points.push_back(point);
        }
    }
    fin.close();
}

sensor_msgs::PointCloud2 get_trajectories_msg(const std::string &file) {
    sensor_msgs::PointCloud2 traj_msg;
    pcl::PointCloud<pcl::PointXYZ> traj_pcl;
    read_trajectories(file, traj_pcl);

    traj_pcl.width = traj_pcl.points.size();
    traj_pcl.height = 1;
    pcl::toROSMsg(traj_pcl, traj_msg);
    traj_msg.header.frame_id = "map";
    traj_msg.header.stamp = ros::Time::now();

    return traj_msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectories comparison");
    ros::NodeHandle node_handle("~");

    std::string trajs_input_archive_1_file;
    std::string trajs_input_archive_2_file;

    node_handle.param("files/trajs_input_archive_1_file",
                      trajs_input_archive_1_file, std::string("/home/galanton/catkin_ws/trajectory"));
    node_handle.param("files/trajs_input_archive_2_file",
                      trajs_input_archive_2_file, std::string("/home/galanton/catkin_ws/trajectory_"));

    sensor_msgs::PointCloud2 trajs_input_archive_1_msg = get_trajectories_msg(trajs_input_archive_1_file);
    sensor_msgs::PointCloud2 trajs_input_archive_2_msg = get_trajectories_msg(trajs_input_archive_2_file);

    ros::Publisher trajs_archive_1_pub = node_handle.advertise<sensor_msgs::PointCloud2>("trajectories_1", 1);
    ros::Publisher trajs_archive_2_pub = node_handle.advertise<sensor_msgs::PointCloud2>("trajectories_2", 1);

    ros::Rate rate(1);
    while (ros::ok()) {
        trajs_input_archive_1_msg.header.stamp = ros::Time::now();
        trajs_input_archive_2_msg.header.stamp = ros::Time::now();
        trajs_archive_1_pub.publish(trajs_input_archive_1_msg);
        trajs_archive_2_pub.publish(trajs_input_archive_2_msg);

        rate.sleep();
    }
}