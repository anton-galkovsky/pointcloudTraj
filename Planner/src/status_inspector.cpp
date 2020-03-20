#include <ros/ros.h>
#include <fstream>
#include <std_msgs/Int8.h>

using namespace std;

static string var_exit_code_file;

//  0    OK
//  1    Can't find a path, mission stall, please reset the target
//  2    Cannot find a feasible and optimal solution, somthing wrong with the mosek solver ...
//  3    Collision Occur, Stop
//  4    max_time_sec has passed
//  5    default
void print_code(int code) {
    ofstream fout(var_exit_code_file, ios::out);
    fout << "EXIT_CODE=" << code << "\n";
    fout.close();
}

void status_code_callback(const std_msgs::Int8 &int_msg) {
    print_code(int_msg.data);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "status inspector");
    ros::NodeHandle node_handle("~");

    ros::Subscriber status_code_sub = node_handle.subscribe("status_code", 50, status_code_callback);

    int max_time_sec;

    node_handle.param("max_time_sec",        max_time_sec,       120);
    node_handle.param("files/var_exit_code", var_exit_code_file, string("/home/galanton/catkin_ws/exit_code"));

    print_code(5);

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