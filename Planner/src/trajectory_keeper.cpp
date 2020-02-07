#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

static std::string trajs_output_archive_file;
static std::string var_traj_length_file;
static pcl::PointCloud<pcl::PointXYZ> *current_trajectory;
static double current_trajectory_length = 0;

void write_trajectory_segment(const pcl::PointCloud<pcl::PointXYZ> &traj_pcl) {
    std::ofstream fout(trajs_output_archive_file, std::ios::app);
    fout << traj_pcl.width << "\n";
    for (pcl::PointXYZ point : traj_pcl.points) {
        fout << point.x << " " << point.y << " " << point.z << "\n";
    }
    fout.close();
}

void append_current_trajectory(const pcl::PointCloud<pcl::PointXYZ> &traj_pcl) {
    for (pcl::PointXYZ point : traj_pcl.points) {
        current_trajectory->push_back(point);
    }
}

double distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
}

void update_current_trajectory_length(const pcl::PointCloud<pcl::PointXYZ> &traj_pcl) {
    for (unsigned long i = 0; i < traj_pcl.points.size() - 1; i++) {
        current_trajectory_length += distance(traj_pcl.points[i], traj_pcl.points[i + 1]);
    }
}

void write_current_trajectory_length(const pcl::PointCloud<pcl::PointXYZ> &traj_pcl) {
    std::ofstream fout(var_traj_length_file, std::ios::out);
    fout << "TRAJ_LENGTH=" << current_trajectory_length << "\n";
    fout.close();
}

void traj_com_pts_callback(const sensor_msgs::PointCloud2 &traj_msg) {
    pcl::PointCloud<pcl::PointXYZ> traj_pcl;
    pcl::fromROSMsg(traj_msg, traj_pcl);

    append_current_trajectory(traj_pcl);
    write_trajectory_segment(traj_pcl);
    update_current_trajectory_length(traj_pcl);
    write_current_trajectory_length(traj_pcl);
}

void publish_current_trajectory(ros::Publisher &oldtrajs_pub) {
    sensor_msgs::PointCloud2 cur_traj_msg;

    current_trajectory->width = current_trajectory->points.size();
    current_trajectory->height = 1;
    pcl::toROSMsg(*current_trajectory, cur_traj_msg);
    cur_traj_msg.header.frame_id = "map";
    cur_traj_msg.header.stamp = ros::Time::now();
    oldtrajs_pub.publish(cur_traj_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory keeper");
    ros::NodeHandle node_handle("~");

    current_trajectory = new pcl::PointCloud<pcl::PointXYZ>();

    node_handle.param("files/trajs_output_archive_file",
            trajs_output_archive_file, std::string("/home/galanton/catkin_ws/trajectory_archive"));
    node_handle.param("files/var_traj_length_file",
            var_traj_length_file, std::string("/home/galanton/catkin_ws/traj_length"));

    ros::Publisher cur_traj_pub =
            node_handle.advertise<sensor_msgs::PointCloud2>("current_trajectory", 1);
    ros::Subscriber traj_com_pts_sub =
            node_handle.subscribe("trajectory_commit_points", 50, traj_com_pts_callback);

    ros::Rate rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        publish_current_trajectory(cur_traj_pub);
    }

    delete current_trajectory;
}