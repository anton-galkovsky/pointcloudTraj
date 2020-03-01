#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <cmath>
#include <ctime>
#include <random>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include "pointcloudTraj/map_observer.h"

using namespace std;

map_observer *observer;
vector<vector<Eigen::Vector3f>> shapes;
//double sense_rate;
int image_width, image_height, fov_horizontal;

//ros::Publisher _all_map_pub, _observed_map_pub;
//
//sensor_msgs::PointCloud2 globalMap_pcd, localMap_pcd, localMap_nofloor_pcd;
//pcl::PointCloud<pcl::PointXYZ> cloudMap, coneMap;
//sensor_msgs::PointCloud2 _observed_map_msg;


void generate_image() {
    float sqrt2_2 = M_SQRT2 / 2;
    Eigen::Vector3f translation(-2, -2, 0.7);
    Eigen::Matrix3f rotation;
    rotation <<   sqrt2_2,    0,  sqrt2_2,
                 -sqrt2_2,    0,  sqrt2_2,
                        0,   -1,        0;

    Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();
    sensorPose.translate(translation);
    sensorPose.rotate(Eigen::AngleAxisf(rotation));

    observer->set_camera_pose(sensorPose);

    float *dense_image = observer->render_to_image();

    unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(dense_image, image_width, image_height);
    pcl::io::saveRgbPNGFile("/home/galanton/catkin_ws/view.png", rgb_image, image_width, image_height);


//    pcl::PointCloud<pcl::PointXYZ> pcd;
//    Eigen::Vector3f p;
//    for (unsigned int i = 0; i < rangeImage.height; i++) {
//        for (unsigned int j = 0; j < rangeImage.width; j++) {
//            rangeImage.calculate3DPoint(j, i, ranges[i * rangeImage.width + j], p);
//            pcd.points.push_back(pcl::PointXYZ(p(0), p(1), p(2)));
//        }
//    }


//    pcd.width = pcd.points.size();
//    pcd.height = 1;
//    pcd.is_dense = true;
//    pcl::toROSMsg(pcd, _observed_map_msg);
//    _observed_map_msg.header.frame_id = "map";
//    _observed_map_pub.publish(_observed_map_msg);
}

void generate_map() {
    vector<Eigen::Vector3f> shape;
    shape.emplace_back(Eigen::Vector3f(-0.5, 0.5, 0));
    shape.emplace_back(Eigen::Vector3f(30, 2, -10));
    shape.emplace_back(Eigen::Vector3f(0, 0, 1.41));
    shapes.push_back(shape);

    ROS_WARN("[Map Controller] Finished generating map");
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "map controller");
    ros::NodeHandle node_handle("~");
//
//    ros::Publisher all_map_publisher =
//            node_handle.advertise<sensor_msgs::PointCloud2>("all_map", 1);
//
//    observed_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/map_server/observed_map", 1);
//
//
//    node_handle.param("sense_rate", sense_rate, 2.0);
    node_handle.param("image_width", image_width, 320);
    node_handle.param("image_height", image_height, 240);
    node_handle.param("fov_horizontal", fov_horizontal, 90);

//    ros::Rate rt(1);
//    for (int i = 0; i < 8; i++)
//        rt.sleep();

    generate_map();
    observer = new map_observer(shapes, image_width, image_height, fov_horizontal);
    generate_image();


//    ros::Rate loop_rate(_sense_rate);
//    while (ros::ok())
//    {
//        pubSensedPoints();
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
}