#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include "pointcloudTraj/utils.h"

using namespace std;

vector<marked_img_pt_pair_type> intersect_sorted_marked_img_pts(
        const vector<marked_img_pt_type> &marked_img_pts_1,
        const vector<marked_img_pt_type> &marked_img_pts_2) {
    vector<marked_img_pt_pair_type> intersection;
    auto it_1 = marked_img_pts_1.begin();
    auto it_2 = marked_img_pts_2.begin();
    while (it_1 != marked_img_pts_1.end() && it_2 != marked_img_pts_2.end()) {
        if (get<0>(*it_1) < get<0>(*it_2)) {
            ++it_1;
        } else if (get<0>(*it_1) > get<0>(*it_2)) {
            ++it_2;
        } else {
            intersection.emplace_back(get<0>(*it_1),
                                      get<1>(*it_1), get<2>(*it_1), get<1>(*it_2), get<2>(*it_2));
            ++it_1;
            ++it_2;
        }
    }
    return intersection;
}

void append_marker_array_msg(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2,
                             double r, double g, double b, double x, double y, bool pointer,
                             visualization_msgs::MarkerArray &marker_array_msg, int id) {
    static int id_ = 0;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "/arrows";
    if (id == -1) {
        arrow.id = id_++;
    } else {
        arrow.id = id;
    }
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    if (x < 0.001) {
        arrow.scale.x = 0.05;
    } else {
        arrow.scale.x = x;
    }
    if (y < 0.001) {
        arrow.scale.y = 0.05;
    } else {
        arrow.scale.y = y;
    }
    if (pointer) {
        arrow.scale.z = 0.0;
    } else {
        arrow.scale.z = 0.01;
    }
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1;

    geometry_msgs::Point p;
    p.x = v1[0];
    p.y = v1[1];
    p.z = v1[2];
    arrow.points.push_back(p);
    p.x = v2[0];
    p.y = v2[1];
    p.z = v2[2];
    arrow.points.push_back(p);
    marker_array_msg.markers.push_back(arrow);
}

template<class T>
void intersect_points_with_plane(list<T> &points, const T &plane_point, const T &normal) {
    for (auto it_1 = points.begin(), it = it_1; it != points.end();) {
        ++it_1;
        if (it_1 == points.end()) {
            it_1 = points.begin();
        }
        double n_dot_i = (*it - plane_point).dot(normal);
        double n_dot_i_1 = (*it_1 - plane_point).dot(normal);
        if ((n_dot_i < -0.01 && n_dot_i_1 > 0.01) || (n_dot_i_1 < -0.01 && n_dot_i > 0.01)) {
            auto new_it = points.insert(it_1, *it - (*it_1 - *it) * n_dot_i / (*it_1 - *it).dot(normal));
            if (new_it == points.begin()) {
                break;
            } else {
                it = it_1;
            }
        } else {
            ++it;
        }
    }
    for (auto it = points.begin(); it != points.end();) {
        if ((*it - plane_point).dot(normal) < -0.01) {
            it = points.erase(it);
        } else {
            ++it;
        }
    }
}

template void intersect_points_with_plane<Eigen::Vector3f>(list<Eigen::Vector3f> &points,
                                                           const Eigen::Vector3f &plane_point,
                                                           const Eigen::Vector3f &normal);
template void intersect_points_with_plane<Eigen::Vector2d>(list<Eigen::Vector2d> &points,
                                                           const Eigen::Vector2d &plane_point,
                                                           const Eigen::Vector2d &normal);

