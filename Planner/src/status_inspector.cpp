#include <ros/ros.h>
#include <fstream>
#include <std_msgs/Int8.h>

static std::string var_exit_code_file;

//  0    OK
//  1    Can't find a path, mission stall, please reset the target
//  2    Cannot find a feasible and optimal solution, somthing wrong with the mosek solver ...
//  3    Collision Occur, Stop
//  4    max_time_s has passed
//  5    default
void print_code(int code) {
    std::ofstream fout(var_exit_code_file, std::ios::out);
    fout << "EXIT_CODE=" << code << "\n";
    fout.close();
}

void statCodeCallback(const std_msgs::Int8 &int_msg) {
    print_code(int_msg.data);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "status inspector");
    ros::NodeHandle node_handle("~");

    node_handle.param("files/var_exit_code_file",
                      var_exit_code_file, std::string("/home/galanton/catkin_ws/exit_code"));

    ros::Subscriber status_code_sub
            = node_handle.subscribe("/sim_planning_demo/status_code", 1, statCodeCallback);

    print_code(5);

    const int frequency = 10;
    const int max_time_s = 100;
    ros::Rate rate(frequency);
    for (int i = 0; i < max_time_s * frequency && ros::ok(); i++) {
        ros::spinOnce();
        rate.sleep();
    }

    if (ros::ok()) {
        print_code(4);
    }
}