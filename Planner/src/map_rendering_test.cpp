#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "pointcloudTraj/map_observer.h"
#include "pointcloudTraj/map_generator.h"

using namespace std;

static string image_file_png;
static string image_file_txt;
static int image_w, image_h;
static double x_init, y_init, z_init;
static double axis_z_0, axis_z_1, axis_z_2;

void prepare_rendering_scan(img_pcl_map_observer &observer, sensor_msgs::PointCloud2 &observed_map_msg,
                            sensor_msgs::Image &image_msg) {
    Eigen::Vector3f camera_axis_z(axis_z_0, axis_z_1, axis_z_2);
    camera_axis_z /= camera_axis_z.norm();

    Eigen::Vector3f camera_axis_x = camera_axis_z.cross(Eigen::Vector3f(0, 0, 1));

    Eigen::Vector3f camera_axis_y = camera_axis_z.cross(camera_axis_x);
    if (camera_axis_y[2] > 0) {
        camera_axis_x *= -1;
        camera_axis_y *= -1;
    }

    Eigen::Vector3f translation(x_init, y_init, z_init);
    Eigen::Matrix3f rotation;
    rotation << camera_axis_x, camera_axis_y, camera_axis_z;

    Eigen::Affine3f camera_pose = Eigen::Affine3f::Identity();
    camera_pose.translate(translation);
    camera_pose.rotate(rotation);

    observer.set_camera_pose(camera_pose);

    auto observed_map_pcl = observer.render_to_pcl();
    pcl::toROSMsg(*observed_map_pcl, observed_map_msg);
    observed_map_msg.header.frame_id = "map";

    auto depth_image = observer.render_to_img();

    auto rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(depth_image, image_w, image_h);
    pcl::io::saveRgbPNGFile(image_file_png, rgb_image, image_w, image_h);

    ofstream fout(image_file_txt);
    for (int j = 0; j < image_h; j++) {
        for (int i = 0; i < image_w; i++) {
            double v = depth_image[j * image_w + i];
            if (v == INFINITY) {
                fout << "    ";
            } else {
                fout.width(4);
                fout << (int) (v * 100) % 10000;
            }
        }
        fout << "\n";
    }
    fout.close();

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "map rendering test");
    ros::NodeHandle node_handle("~");

    ros::Publisher all_map_pub      = node_handle.advertise<sensor_msgs::PointCloud2>("all_map", 1);
    ros::Publisher observed_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("observed_map", 1);
    ros::Publisher image_pub        = node_handle.advertise<sensor_msgs::Image>("observed_map_image", 1);

    double x_end, y_end, x_1, x_2, y_1, y_2, w_1, w_2, h_1, h_2, res;
    int obs_num, seed, fov_hor;

    node_handle.param("map_boundary/lower_x", x_1,     -50.0);
    node_handle.param("map_boundary/upper_x", x_2,      50.0);
    node_handle.param("map_boundary/lower_y", y_1,     -50.0);
    node_handle.param("map_boundary/upper_y", y_2,      50.0);

    node_handle.param("obstacles/lower_w",    w_1,       0.6);
    node_handle.param("obstacles/upper_w",    w_2,       3.2);
    node_handle.param("obstacles/lower_h",    h_1,       1.0);
    node_handle.param("obstacles/upper_h",    h_2,      10.0);

    node_handle.param("map/resolution",       res,       0.1);
    node_handle.param("map/obstacles_num",    obs_num,   600);
    node_handle.param("map/seed",             seed,      1);

    node_handle.param("camera/width",         image_w,  1280);
    node_handle.param("camera/height",        image_h,  960);
    node_handle.param("camera/fov_hor",       fov_hor,  90);

    node_handle.param("copter/init_x",        x_init,  -45.0);
    node_handle.param("copter/init_y",        y_init,  -45.0);
    node_handle.param("copter/init_z",        z_init,    2.0);
    node_handle.param("copter/end_x",         x_end,    30.0);
    node_handle.param("copter/end_y",         y_end,    30.0);
    node_handle.param("copter/axis_z_0",      axis_z_0,  0.707);
    node_handle.param("copter/axis_z_1",      axis_z_1,  0.707);
    node_handle.param("copter/axis_z_2",      axis_z_2,  0.0);

    node_handle.param("files/rendered_image_png", image_file_png, string("/home/galanton/catkin_ws/view.png"));
    node_handle.param("files/rendered_image_txt", image_file_txt, string("/home/galanton/catkin_ws/view_txt"));

    map_generator generator(x_init, x_end, y_init, y_end, x_1, x_2, y_1, y_2, h_1, h_2, w_1, w_2, res, obs_num, seed);
    generator.generate_map();

    auto global_map_pcl = generator.get_global_map_pcl();
    sensor_msgs::PointCloud2 global_map_msg;
    pcl::toROSMsg(global_map_pcl, global_map_msg);
    global_map_msg.header.frame_id = "map";

    auto shapes = generator.get_shapes();
    auto observer = img_pcl_map_observer(shapes, image_w, image_h, fov_hor, 1000);

    sensor_msgs::PointCloud2 observed_map_msg;
    sensor_msgs::Image image_msg;

    prepare_rendering_scan(observer, observed_map_msg, image_msg);

    ros::Rate loop_rate(1);
    for (int i = 0; i < 10 && ros::ok(); i++) {
        all_map_pub.publish(global_map_msg);
        observed_map_pub.publish(observed_map_msg);
        image_pub.publish(image_msg);
        loop_rate.sleep();
    }
}