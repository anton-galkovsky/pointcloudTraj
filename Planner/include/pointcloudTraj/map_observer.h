#pragma once

#include <vector>
#include <tuple>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class map_observer {
public:
    class plane_convex_shape {
    public:
        explicit plane_convex_shape(const std::vector<Eigen::Vector3d> &sp_points);

        std::vector<Eigen::Vector3d> spatial_points;
        std::vector<Eigen::Vector3d> image_points;
        Eigen::Vector3d normal;
    };

    virtual void set_camera_pose(const Eigen::Affine3f &camera_pose);

    static double get_focal_distance(double fov_hor);

protected:
    map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                 int img_width, int img_height, int fov_hor, double max_distance_z);

    void render();

    virtual void operate_marked(int shape_idx);

    virtual void save_point(int x, int y, double d, map_observer::plane_convex_shape * shape_ptr) = 0;

    virtual ~map_observer() = default;

    std::vector<plane_convex_shape> shapes;

    Eigen::Vector3d camera_axis_x;
    Eigen::Vector3d camera_axis_y;
    Eigen::Vector3d camera_axis_z;
    Eigen::Vector3d camera_translation;

    Eigen::Vector3d cone_normals[4];

    int image_width;
    int image_height;
    double focal_distance;

private:
    bool compute_points_in_cone(const std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &cone_points);

    void recount_cone_params();

    static void intersect_points(std::list<Eigen::Vector3d> &points,
                                 const Eigen::Vector3d &plane_point, const Eigen::Vector3d &normal);

    double hor_angle_2;
    double ver_angle_2;

    double max_distance_z;
};

typedef std::tuple<int, double, int, double> segment_type;
typedef map_observer::plane_convex_shape * shape_ptr_type;
typedef std::tuple<int, double, int, double, shape_ptr_type> marked_segment_type;

class img_map_observer : public map_observer {
public:
    img_map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                     int img_width, int img_height, int fov_hor, double max_distance_z);

    const float *render_to_img();

    ~img_map_observer() override;

private:
    void save_point(int x, int y, double d, shape_ptr_type shape_ptr) override;

    float *image;
};

class pcl_map_observer : public map_observer {
public:
    pcl_map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                     int img_width, int img_height, int fov_hor, double max_distance_z);

    const pcl::PointCloud<pcl::PointXYZ> *render_to_pcl();

    ~pcl_map_observer() override = default;

private:
    void save_point(int x, int y, double d, shape_ptr_type shape_ptr) override;

    pcl::PointCloud<pcl::PointXYZ> pcl;
};

class img_pcl_map_observer : public map_observer {
public:
    img_pcl_map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                         int img_width, int img_height, int fov_hor, double max_distance_z);

    void set_camera_pose(const Eigen::Affine3f &camera_pose) override;

    const float *render_to_img();

    const pcl::PointCloud<pcl::PointXYZ> *render_to_pcl();

    ~img_pcl_map_observer() override;

private:
    void save_point(int x, int y, double d, shape_ptr_type shape_ptr) override;

    float *image;
    pcl::PointCloud<pcl::PointXYZ> pcl;

    bool actual_rendering_data;
};

class marked_map_observer : public map_observer {
public:
    marked_map_observer(std::vector<std::vector<int>> marked_point_indexes_arr,
                        std::vector<Eigen::Vector3d> marked_points_vec,
                        const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                        int img_width, int img_height, int fov_hor, double max_distance_z);

    void set_camera_pose(const Eigen::Affine3f &camera_pose) override;

    const std::vector<std::tuple<const Eigen::Vector3d *, int, int>> &render_to_marked_img_pts();

    const float *render_to_img();

    double get_pixel_cone_angle_2();

    ~marked_map_observer() override;

private:
    void save_point(int x, int y, double d, shape_ptr_type shape_ptr) override;

    void operate_marked(int shape_idx) override;

    std::vector<std::vector<int>> marked_point_indexes_arr;
    std::vector<Eigen::Vector3d> marked_points_vec;
    std::vector<std::tuple<const Eigen::Vector3d *, int, int>> marked_img_pts;
    std::tuple<const Eigen::Vector3d *, shape_ptr_type, float> *marks_image;

    float *image;

    double pixel_cone_angle_2;

    bool actual_rendering_data;
};