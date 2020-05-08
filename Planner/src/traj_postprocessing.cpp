#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_msgs/PolynomialTrajectoryYawed.h>
#include <quadrotor_msgs/PolynomialTrajectoryExtra.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <pointcloudTraj/voxel_map.h>
#include <pointcloudTraj/binomial_coefs.h>

using namespace std;

ros::Publisher poly_traj_pub, shift_ctrl_pts_pub, traj_segm_targ_pub, nearest_traj_pub;

bool has_intact_traj;
int poly_order_max;
double res, twirl_len;
binomial_coefs bins;
Eigen::Vector3d cur_pos, cur_vel;
quadrotor_msgs::PolynomialTrajectoryExtra traj_extra;

Eigen::Vector3d coef_vec(const quadrotor_msgs::PolynomialTrajectoryExtra &traj_ext, int i) {
    return Eigen::Vector3d(traj_ext.coef_x[i], traj_ext.coef_y[i], traj_ext.coef_z[i]);
}

pair<int, int> get_segm_index(const quadrotor_msgs::PolynomialTrajectoryExtra &traj_ext) {
    double len = twirl_len;

    Eigen::Vector3d last_p = coef_vec(traj_ext, 0) * traj_ext.time[0];
    int order_shift = 0;
    for (int segm = 0; segm < (int) traj_ext.num_segment; segm++) {
        int order = traj_ext.order[segm];
        for (int i = 0; i < 1001; i++) {
            double t = i / 1000.0;

            Eigen::Vector3d p(0, 0, 0);
            for (int k = 0; k < order + 1; k++) {
                p += traj_ext.time[segm] * coef_vec(traj_ext, order_shift + k) *
                     bins.c(order, k) * pow(t, k) * pow(1 - t, order - k);
            }

            len -= (p - last_p).norm();
            last_p = p;

            if (len < 0) {
                return {segm, t > 0.5 ? 1 : 0};
            }
        }
        order_shift += order + 1;
    }
    return {traj_ext.num_segment - 1, 1};
}

sensor_msgs::PointCloud2 to_nearest_traj(const quadrotor_msgs::PolynomialTrajectoryExtra &traj_ext) {
    double len = twirl_len;

    voxel_map_pcl traj_voxels(res);
    if (traj_ext.num_segment > 0) {
        Eigen::Vector3d last_p = coef_vec(traj_ext, 0) * traj_ext.time[0];
        int order_shift = 0;
        for (int segm = 0; segm < (int) traj_ext.num_segment && len >= 0; segm++) {
            int order = traj_ext.order[segm];
            for (int i = 0; i < 1001 && len >= 0; i++) {
                double t = i / 1000.0;

                Eigen::Vector3d p(0, 0, 0);
                for (int k = 0; k < order + 1; k++) {
                    p += traj_ext.time[segm] * coef_vec(traj_ext, order_shift + k) *
                         bins.c(order, k) * pow(t, k) * pow(1 - t, order - k);
                }

                traj_voxels.add_point(p);
                len -= (p - last_p).norm();
                last_p = p;
            }
            order_shift += order + 1;
        }
    }
    auto traj_pcl = traj_voxels.get_voxel_cloud();
    traj_pcl.height = 1;
    traj_pcl.is_dense = true;
    traj_pcl.width = traj_pcl.size();
    sensor_msgs::PointCloud2 traj_msg;
    pcl::toROSMsg(traj_pcl, traj_msg);
    traj_msg.header.frame_id = "map";
    return traj_msg;
}

visualization_msgs::MarkerArray empty_shift_ctrl_pts() {
    visualization_msgs::MarkerArray ctrl_pts;

    visualization_msgs::Marker pt;
    pt.header.frame_id = "map";
    pt.action = visualization_msgs::Marker::DELETE;
    for (int i = 0; i < poly_order_max + 1; i++) {
        pt.id = i;
        ctrl_pts.markers.push_back(pt);
    }

    return ctrl_pts;
}

visualization_msgs::MarkerArray to_shift_ctrl_pts(const quadrotor_msgs::PolynomialTrajectoryExtra &traj_ext) {
    visualization_msgs::MarkerArray ctrl_pts;

    visualization_msgs::Marker pt;
    pt.header.frame_id = "map";
    pt.type = visualization_msgs::Marker::SPHERE;
    pt.action = visualization_msgs::Marker::ADD;
    pt.pose.orientation.x = 0.0;
    pt.pose.orientation.y = 0.0;
    pt.pose.orientation.z = 0.0;
    pt.pose.orientation.w = 1.0;
    pt.color.a = 1.0;
    pt.color.r = 1.0;
    pt.color.g = 1.0;
    pt.color.b = 0.0;

    for (int i = 0; i < poly_order_max + 1; i++) {
        pt.id = i;
        pt.pose.position.x = traj_ext.coef_x[i] * traj_ext.time[0];
        pt.pose.position.y = traj_ext.coef_y[i] * traj_ext.time[0];
        pt.pose.position.z = traj_ext.coef_z[i] * traj_ext.time[0];
        pt.scale.x = 0.25;
        pt.scale.y = 0.25;
        pt.scale.z = 0.25;
        ctrl_pts.markers.push_back(pt);
    }

    return ctrl_pts;
}

quadrotor_msgs::PolynomialTrajectoryYawed to_poly_traj(const quadrotor_msgs::PolynomialTrajectoryExtra &traj_ext) {
    quadrotor_msgs::PolynomialTrajectoryYawed traj;

    traj.header        = traj_ext.header;
    traj.trajectory_id = traj_ext.trajectory_id;
    traj.action        = traj_ext.action;
    traj.num_order     = traj_ext.num_order;
    traj.num_segment   = traj_ext.num_segment;
    traj.start_yaw     = traj_ext.start_yaw;
    traj.final_yaw     = traj_ext.final_yaw;
    traj.coef_x        = traj_ext.coef_x;
    traj.coef_y        = traj_ext.coef_y;
    traj.coef_z        = traj_ext.coef_z;
    traj.time          = traj_ext.time;
    traj.mag_coeff     = traj_ext.mag_coeff;
    traj.order         = traj_ext.order;
    traj.debug_info    = traj_ext.debug_info;


    if (!traj_ext.radii.empty()) {
        traj.end_yaws = vector<double>(traj_ext.radii.size());
        for (int i = 0; i < (int) traj_ext.radii.size(); i++) {
            if (i < (int) traj_ext.radii.size() - 1) {
                Eigen::Vector2d v(traj_ext.path_x[i + 1] - traj_ext.path_x[i],
                                  traj_ext.path_y[i + 1] - traj_ext.path_y[i]);
                if (v.norm() > 0.01) {
                    traj.end_yaws[i] = atan2(v[1], v[0]);
                } else {
                    traj.end_yaws[i] = 10;
                }
            } else {
                if (i > 0) {
                    traj.end_yaws[i] = traj.end_yaws[i - 1];
                } else {
                    Eigen::Vector2d v(traj_ext.coef_x[1] - traj_ext.coef_x[0],
                                      traj_ext.coef_y[1] - traj_ext.coef_y[0]);
                    if (v.norm() > 0.01) {
                        traj.end_yaws[i] = atan2(v[1], v[0]);
                    } else {
                        traj.end_yaws[i] = 10;
                    }
                }
            }
        }

        traj.middle_yaws = vector<double>(traj.end_yaws.begin(), traj.end_yaws.end());
    }

    return traj;
}

void shift_cmd_callback(const geometry_msgs::Vector3 &shift_vec_msg) {
    if (!has_intact_traj || traj_extra.num_segment == 0) {
        return;
    }
    has_intact_traj = false;
    ROS_WARN("[Post] Execute shift");

    Eigen::Vector3d shift_vec(shift_vec_msg.x, shift_vec_msg.y, shift_vec_msg.z);

    int cur_segm = 0;
    while ((cur_pos - Eigen::Vector3d(traj_extra.path_x[cur_segm], traj_extra.path_y[cur_segm],
                                      traj_extra.path_z[cur_segm])).norm() > traj_extra.radii[cur_segm]) {
        cur_segm++;
    }
    while (++cur_segm < (int) traj_extra.num_segment
           && (cur_pos - Eigen::Vector3d(traj_extra.path_x[cur_segm], traj_extra.path_y[cur_segm],
                                         traj_extra.path_z[cur_segm])).norm() <= traj_extra.radii[cur_segm]);
    cur_segm--;

    int orders_erase_num_ = 0;
    for (int i = 0; i < (int) traj_extra.num_segment; i++) {
        orders_erase_num_ += (int) traj_extra.order[i] + 1;
    }

    traj_extra.num_segment -= cur_segm;
    traj_extra.time = vector<double>(traj_extra.time.begin() + cur_segm, traj_extra.time.end());
    traj_extra.radii = vector<double>(traj_extra.radii.begin() + cur_segm, traj_extra.radii.end());
    traj_extra.path_x = vector<double>(traj_extra.path_x.begin() + cur_segm, traj_extra.path_x.end());
    traj_extra.path_y = vector<double>(traj_extra.path_y.begin() + cur_segm, traj_extra.path_y.end());
    traj_extra.path_z = vector<double>(traj_extra.path_z.begin() + cur_segm, traj_extra.path_z.end());

    int orders_erase_num = 0;
    for (int i = 0; i < cur_segm; i++) {
        orders_erase_num += (int) traj_extra.order[i] + 1;
    }
    traj_extra.order = vector<uint32_t>(traj_extra.order.begin() + cur_segm, traj_extra.order.end());

    int pts_num = poly_order_max + 1;
    orders_erase_num += (int) traj_extra.order[0] + 1 - pts_num;
    if (orders_erase_num >= 0) {
        traj_extra.coef_x = vector<double>(traj_extra.coef_x.begin() + orders_erase_num, traj_extra.coef_x.end());
        traj_extra.coef_y = vector<double>(traj_extra.coef_y.begin() + orders_erase_num, traj_extra.coef_y.end());
        traj_extra.coef_z = vector<double>(traj_extra.coef_z.begin() + orders_erase_num, traj_extra.coef_z.end());
    } else {
        for (int i = 0; i < -orders_erase_num; i++) {
            traj_extra.coef_x.insert(traj_extra.coef_x.begin(), 0);
            traj_extra.coef_y.insert(traj_extra.coef_y.begin(), 0);
            traj_extra.coef_z.insert(traj_extra.coef_z.begin(), 0);
        }
    }

    traj_extra.order[0] = poly_order_max;

    Eigen::Vector3d center(traj_extra.path_x[0], traj_extra.path_y[0], traj_extra.path_z[0]);
    double radius = traj_extra.radii[0];

    traj_extra.time[0] = 7;


    double vel_init_scale = traj_extra.time[0] / traj_extra.order[0];
    double vel_scale = vel_init_scale;
    while ((cur_pos + cur_vel * vel_scale - center).norm() > radius) {
        vel_scale *= 0.9;
    }
    Eigen::Vector3d pos_vel = cur_pos + cur_vel * vel_scale;


    traj_extra.coef_x[0] = cur_pos[0];
    traj_extra.coef_y[0] = cur_pos[1];
    traj_extra.coef_z[0] = cur_pos[2];

    traj_extra.coef_x[1] = pos_vel[0];
    traj_extra.coef_y[1] = pos_vel[1];
    traj_extra.coef_z[1] = pos_vel[2];


    Eigen::Vector3d next_pos, end_vel;
    if (traj_extra.num_segment > 1) {
        end_vel = traj_extra.order[1] * (coef_vec(traj_extra, pts_num + 1) - coef_vec(traj_extra, pts_num));
        next_pos = coef_vec(traj_extra, pts_num) * traj_extra.time[1];
    } else {
        next_pos = coef_vec(traj_extra, poly_order_max);
        end_vel << 0, 0, 0;
    }


    double end_vel_scale = vel_init_scale;
    while ((next_pos - end_vel * end_vel_scale - center).norm() > radius) {
        end_vel_scale *= 0.9;
    }
    Eigen::Vector3d end_pos_vel = next_pos - end_vel * end_vel_scale;

    traj_extra.coef_x[pts_num - 2] = end_pos_vel[0];
    traj_extra.coef_y[pts_num - 2] = end_pos_vel[1];
    traj_extra.coef_z[pts_num - 2] = end_pos_vel[2];

    traj_extra.coef_x[pts_num - 1] = next_pos[0];
    traj_extra.coef_y[pts_num - 1] = next_pos[1];
    traj_extra.coef_z[pts_num - 1] = next_pos[2];


    Eigen::Vector3d rotation_axis = shift_vec.cross(cur_vel).cross(shift_vec).normalized();
    Eigen::Vector3d rotation_vec = shift_vec;
    if (rotation_vec.norm() > radius) {
        rotation_vec = rotation_vec.normalized() * radius;
    } else if (rotation_vec.norm() < radius / 2) {
        rotation_vec = rotation_vec.normalized() * radius / 2;
    }

    int circle_points_num = pts_num - 4;
    double circle_angle = 2 * M_PI / circle_points_num;
    Eigen::AngleAxisd circle_rotation(circle_angle, rotation_axis);
    for (int j = 0; j < circle_points_num; j++) {
        traj_extra.coef_x[j + 2] = rotation_vec(0) + center(0);
        traj_extra.coef_y[j + 2] = rotation_vec(1) + center(1);
        traj_extra.coef_z[j + 2] = rotation_vec(2) + center(2);

        rotation_vec = circle_rotation * rotation_vec;
    }


    for (int i = 0; i < pts_num; i++) {
        traj_extra.coef_x[i] /= traj_extra.time[0];
        traj_extra.coef_y[i] /= traj_extra.time[0];
        traj_extra.coef_z[i] /= traj_extra.time[0];
    }

    traj_extra.header.stamp = ros::Time::now();

    poly_traj_pub.publish(to_poly_traj(traj_extra));
    nearest_traj_pub.publish(to_nearest_traj(traj_extra));
    shift_ctrl_pts_pub.publish(to_shift_ctrl_pts(traj_extra));
}

void twirl_cmd_callback(const std_msgs::Float64 &twirl_yaw_msg) {
    if (!has_intact_traj || traj_extra.num_segment == 0) {
        return;
    }
    has_intact_traj = false;
    ROS_WARN("[Post] Execute twirl");

    double target_yaw = twirl_yaw_msg.data;

    auto poly_traj = to_poly_traj(traj_extra);

    int last_idx, segm_part;
    tie(last_idx, segm_part) = get_segm_index(traj_extra);
    last_idx--;

    for (int i = 0; i < last_idx; i++) {
        poly_traj.middle_yaws[i] = target_yaw;
        poly_traj.end_yaws[i] = target_yaw;
    }
    if (last_idx >= 0) {
        poly_traj.middle_yaws[last_idx] = target_yaw;
        if (segm_part == 1) {
            poly_traj.end_yaws[last_idx] = target_yaw;
        }
    }

    poly_traj_pub.publish(poly_traj);
}

void poly_traj_extra_callback(const quadrotor_msgs::PolynomialTrajectoryExtra &traj_msg) {
    has_intact_traj = true;
    traj_extra = traj_msg;
    poly_traj_pub.publish(to_poly_traj(traj_extra));
    nearest_traj_pub.publish(to_nearest_traj(traj_extra));
//    shift_ctrl_pts_pub.publish(empty_shift_ctrl_pts());
}

void position_callback(const quadrotor_msgs::PositionCommand &cmd) {
    cur_pos << cmd.position.x, cmd.position.y, cmd.position.z;
    cur_vel << cmd.velocity.x, cmd.velocity.y, cmd.velocity.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "traj postprocessing");
    ros::NodeHandle node_handle("~");

    ros::Subscriber shift_cmd_sub       = node_handle.subscribe("shift_cmd",       1, shift_cmd_callback);
    ros::Subscriber twirl_cmd_sub       = node_handle.subscribe("twirl_cmd",       1, twirl_cmd_callback);
    ros::Subscriber position_sub        = node_handle.subscribe("position",        1, position_callback);
    ros::Subscriber poly_traj_extra_sub = node_handle.subscribe("poly_traj_extra", 1, poly_traj_extra_callback);

    poly_traj_pub      = node_handle.advertise<quadrotor_msgs::PolynomialTrajectoryYawed>("poly_traj",     10);
    nearest_traj_pub   = node_handle.advertise<sensor_msgs::PointCloud2>                 ("nearest_traj",   1);
    shift_ctrl_pts_pub = node_handle.advertise<visualization_msgs::MarkerArray>          ("shift_ctrl_pts", 1);

    node_handle.param("optimization/poly_order_max", poly_order_max,   10);
    node_handle.param("map/resolution",              res,              0.1);
    node_handle.param("safety/twirl_len",            twirl_len,        1.5);

    has_intact_traj = false;

    ros::spin();
}