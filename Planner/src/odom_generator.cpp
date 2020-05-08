#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"

static double x_init, y_init, z_init, yaw_init;
static bool was_pos;
static quadrotor_msgs::PositionCommand pos;

template<class T>
void copy_xyz(T &dest, double x, double y, double z) {
    dest.x = x;
    dest.y = y;
    dest.z = z;
}

void prepare_odom(nav_msgs::Odometry &odom) {
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "uav";

    if (was_pos) {
        odom.pose.pose.position = pos.position;
        odom.twist.twist.linear = pos.velocity;
        odom.twist.twist.angular = pos.acceleration;
    } else {
        copy_xyz(odom.pose.pose.position, x_init, y_init, z_init);
        copy_xyz(odom.twist.twist.linear, 0, 0, 0);
        copy_xyz(odom.twist.twist.angular, 0, 0, 0);
    }

    odom.pose.pose.orientation.w = yaw_init;
}

void position_callback(const quadrotor_msgs::PositionCommand pos_msg) {
    was_pos = true;
    pos = pos_msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom generator");
    ros::NodeHandle node_handle("~");

    double x_end, y_end, rate;

    node_handle.param("copter/init_x", x_init, -45.0);
    node_handle.param("copter/init_y", y_init, -45.0);
    node_handle.param("copter/init_z", z_init,   2.0);
    node_handle.param("copter/end_x",  x_end,   30.0);
    node_handle.param("copter/end_y",  y_end,   30.0);

    node_handle.param("odom/rate",     rate,   100.0);

    ros::Subscriber position_sub = node_handle.subscribe("position", 1, position_callback);

    ros::Publisher odometry_pub = node_handle.advertise<nav_msgs::Odometry>("odometry", 1);

    yaw_init = atan2(y_end - y_init, x_end - x_init);
    was_pos = false;

    nav_msgs::Odometry odometry_msg;

    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        prepare_odom(odometry_msg);
        odometry_pub.publish(odometry_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}