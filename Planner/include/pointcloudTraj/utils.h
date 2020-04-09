#pragma once

#include <list>
#include <visualization_msgs/MarkerArray.h>

typedef std::tuple<const Eigen::Vector3d *, int, int> marked_img_pt_type;
typedef std::tuple<const Eigen::Vector3d *, int, int, int, int> marked_img_pt_pair_type;

std::vector<marked_img_pt_pair_type> intersect_sorted_marked_img_pts(
        const std::vector<marked_img_pt_type> &marked_img_pts_1,
        const std::vector<marked_img_pt_type> &marked_img_pts_2);

void append_marker_array_msg(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2,
                             double r, double g, double b, double x, double y, bool pointer,
                             visualization_msgs::MarkerArray &marker_array_msg, int id = -1);

template<class T>
void intersect_points_with_plane(std::list<T> &points, const T &plane_point, const T &normal);