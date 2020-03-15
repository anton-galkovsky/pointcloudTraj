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
#include "quadrotor_msgs/PositionCommand.h"

using namespace std;

img_pcl_map_observer *observer;
vector<vector<Eigen::Vector3d>> shapes;
bool was_odom;

double sense_rate;
std::string image_file;
int image_width, image_height, fov_horizontal;
bool only_rendering;

pcl::PointCloud<pcl::PointXYZ> *global_map_pcl;
const pcl::PointCloud<pcl::PointXYZ> *observed_map_pcl;
sensor_msgs::PointCloud2 *global_map_msg, *observed_map_msg;


void emplace_rect_to_map(double x_1, double y_1, double x_2, double y_2, double h, double resolution) {
    int x_1_cell = (int) (x_1 / resolution);
    int y_1_cell = (int) (y_1 / resolution);
    int x_2_cell = (int) (x_2 / resolution);
    int y_2_cell = (int) (y_2 / resolution);
    int h_cell = (int) (h / resolution);

    int x_incr = (x_1_cell < x_2_cell) ? 1 : -1;
    int y_incr = (y_1_cell < y_2_cell) ? 1 : -1;
    if (x_1_cell == x_2_cell) {
        for (int y_i = y_1_cell; y_i != y_2_cell; y_i += y_incr) {
            for (int h_i = 1; h_i < h_cell; h_i++) {
                global_map_pcl->emplace_back(x_1_cell * resolution, y_i * resolution, h_i * resolution);
            }
        }
    } else if (y_1_cell == y_2_cell) {
        for (int x_i = x_1_cell; x_i != x_2_cell; x_i += x_incr) {
            for (int h_i = 1; h_i < h_cell; h_i++) {
                global_map_pcl->emplace_back(x_i * resolution, y_1_cell * resolution, h_i * resolution);
            }
        }
    } else {
        for (int x_i = x_1_cell; x_i != x_2_cell + x_incr; x_i += x_incr) {
            for (int y_i = y_1_cell; y_i != y_2_cell + y_incr; y_i += y_incr) {
                global_map_pcl->emplace_back(x_i * resolution, y_i * resolution, h_cell * resolution);
            }
        }
    }
}

void generate_map() {
//    random_device rd;
//    default_random_engine engine(rd());
    default_random_engine engine(0);

    auto random_x = uniform_real_distribution<double>(-10, 10);
    auto random_y = uniform_real_distribution<double>(-10, 10);
    auto random_w = uniform_real_distribution<double>(0.3, 0.8);
    auto random_h = uniform_real_distribution<double>(3, 7);
    double resolution = 0.1;

    vector<Eigen::Vector3d> cylinders;
    for (int i = 0; i < 30; i++) {
        double x, y, w, h;
        x = random_x(engine);
        y = random_y(engine);
        w = random_w(engine);
        h = random_h(engine);

        bool ok = true;
        for (auto cylinder : cylinders) {
            if (pow(cylinder[0] - x, 2) + pow(cylinder[1] - y, 2) < pow(cylinder[2] + w, 2)) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            continue;
        }
        cylinders.emplace_back(x, y, w);

        vector<Eigen::Vector3d> shape_cap1;
        vector<Eigen::Vector3d> shape_cap2;
        vector<Eigen::Vector3d> shape_element;
        for (int delta = 90, phi = delta / 2; phi < 360 + delta / 2; phi += delta) {
            double x_1 = round((x + w * cos(M_PI / 180 * phi)) / resolution) * resolution;
            double y_1 = round((y + w * sin(M_PI / 180 * phi)) / resolution) * resolution;
            double x_2 = round((x + w * cos(M_PI / 180 * (phi + delta))) / resolution) * resolution;
            double y_2 = round((y + w * sin(M_PI / 180 * (phi + delta))) / resolution) * resolution;

            shape_element.clear();
            shape_element.emplace_back(x_1, y_1, 0);
            shape_element.emplace_back(x_2, y_2, 0);
            shape_element.emplace_back(x_2, y_2, h);
            shape_element.emplace_back(x_1, y_1, h);
            shapes.push_back(shape_element);

            shape_cap1.emplace_back(x_1, y_1, 0);
            shape_cap2.emplace_back(x_1, y_1, h);

            emplace_rect_to_map(x_1, y_1, x_2, y_2, h, resolution);
        }

        shapes.push_back(shape_cap1);
        shapes.push_back(shape_cap2);

        emplace_rect_to_map(shape_cap1[0][0], shape_cap1[0][1],
                            shape_cap1[2][0], shape_cap1[2][1], 0, resolution);
        emplace_rect_to_map(shape_cap1[0][0], shape_cap1[0][1],
                            shape_cap1[2][0], shape_cap1[2][1], h, resolution);
    }

    global_map_pcl->width = global_map_pcl->points.size();
    global_map_pcl->height = 1;
    global_map_pcl->is_dense = true;

    ROS_WARN("[Map Controller] Finished generating map");
}

void position_msg_callback(const quadrotor_msgs::PositionCommand &cmd) {
    was_odom = true;

    Eigen::Vector3f camera_axis_z(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
    camera_axis_z /= camera_axis_z.norm();
    Eigen::Vector3f camera_axis_x(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);
    camera_axis_x -= camera_axis_z * camera_axis_x.dot(camera_axis_z);
    camera_axis_x /= camera_axis_x.norm();
    Eigen::Vector3f camera_axis_y = camera_axis_z.cross(camera_axis_x);
    if (camera_axis_y[2] > 0) {
        camera_axis_x *= -1;
        camera_axis_y *= -1;
    }

    Eigen::Vector3f translation(cmd.position.x, cmd.position.y, cmd.position.z);
    Eigen::Matrix3f rotation;
    rotation << camera_axis_x[0], camera_axis_y[0], camera_axis_z[0],
            camera_axis_x[1], camera_axis_y[1], camera_axis_z[1],
            camera_axis_x[2], camera_axis_y[2], camera_axis_z[2];


    Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();
    sensorPose.translate(translation);
    sensorPose.rotate(Eigen::AngleAxisf(rotation));

    observer->set_camera_pose(sensorPose);
}

void prepare_new_scan() {
    if (was_odom) {
        observed_map_pcl = observer->render_to_pcl();
        pcl::toROSMsg(*observed_map_pcl, *observed_map_msg);
        observed_map_msg->header.frame_id = "map";
    }
}

void prepare_only_rendering_scan() {
    float sqrt2_2 = (float) sqrt(2) / 2;
    Eigen::Vector3f translation(-12, -12, 8);
    Eigen::Matrix3f rotation;
    rotation <<   sqrt2_2,    0,  sqrt2_2,
                 -sqrt2_2,    0,  sqrt2_2,
                        0,   -1,        0;

    Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();
    sensorPose.translate(translation);
    sensorPose.rotate(Eigen::AngleAxisf(rotation));

    observer->set_camera_pose(sensorPose);

    observed_map_pcl = observer->render_to_pcl();
    pcl::toROSMsg(*observed_map_pcl, *observed_map_msg);
    observed_map_msg->header.frame_id = "map";

    const float *depth_image = observer->render_to_img();
    unsigned char *rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(
            depth_image, image_width, image_height);
    pcl::io::saveRgbPNGFile(image_file, rgb_image, image_width, image_height);

    FILE *fout = fopen("/home/galanton/catkin_ws/view__txt", "w");
    for (int j = 0; j < image_height; j++) {
        for (int i = 0; i < image_width; i++) {
            double v = depth_image[j * image_width + i];
//            if (v == INFINITY) {
//                fprintf(fout, " ");
//            } else {
//                fprintf(fout, "%X", ((int) v) % 16);
//            }
            if (v == INFINITY) {
                fprintf(fout, "    ");
            } else {
                fprintf(fout, "%4d", (int) (v * 100));
            }
        }
        fprintf(fout, "\n");
    }
    fclose(fout);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map controller");
    ros::NodeHandle node_handle("~");

    global_map_pcl = new pcl::PointCloud<pcl::PointXYZ>();
    global_map_msg = new sensor_msgs::PointCloud2();
    observed_map_msg = new sensor_msgs::PointCloud2();
    was_odom = false;

    ros::Publisher all_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("all_map", 1);
    ros::Publisher observed_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("observed_map", 1);

    ros::Subscriber position_sub = node_handle.subscribe("position", 50, position_msg_callback);

    node_handle.param("sense_rate", sense_rate, 2.0);
    node_handle.param("image_width", image_width, 320);
    node_handle.param("image_height", image_height, 240);
    node_handle.param("fov_horizontal", fov_horizontal, 90);
    node_handle.param("only_rendering", only_rendering, false);
    node_handle.param("files/rendered_image", image_file,
                      std::string("/home/galanton/catkin_ws/view.png"));

//    ros::Rate rt(1);
//    for (int i = 0; i < 10; i++)
//        rt.sleep();

    generate_map();

    pcl::toROSMsg(*global_map_pcl, *global_map_msg);
    global_map_msg->header.frame_id = "map";

    observer = new img_pcl_map_observer(shapes, image_width, image_height, fov_horizontal);

    ros::Rate loop_rate(sense_rate);
    if (only_rendering) {
        prepare_only_rendering_scan();
        for (int global_map_counter = 0; ros::ok() && global_map_counter < 10; global_map_counter++) {
            all_map_pub.publish(*global_map_msg);
            observed_map_pub.publish(*observed_map_msg);
            loop_rate.sleep();
        }
    } else {
        for (int global_map_counter = 0; ros::ok(); global_map_counter++) {
            if (global_map_counter < 10) {
                all_map_pub.publish(*global_map_msg);
            }

            prepare_new_scan();
            observed_map_pub.publish(*observed_map_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    delete observer;
    delete global_map_pcl;
    delete global_map_msg;
    delete observed_map_msg;
}