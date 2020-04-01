#pragma once

#include <vector>
#include <tuple>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef std::tuple<int, double, int, double> segment_type;

class map_observer {
public:
    virtual void set_camera_pose(const Eigen::Affine3f &camera_pose);

protected:
    map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_, int img_width, int img_height, int fov_hor);

    void render();

    virtual void operate_marked(int shape_idx);

    virtual void save_point(int x, int y, double d) = 0;

    virtual ~map_observer() = default;

    Eigen::Vector3d camera_axis_x;
    Eigen::Vector3d camera_axis_y;
    Eigen::Vector3d camera_axis_z;
    Eigen::Vector3d camera_translation;

    Eigen::Vector3d cone_normals[4];

    int image_width;
    int image_height;
    double focal_distance;

private:
    class plane_convex_shape {
    public:
        explicit plane_convex_shape(const std::vector<Eigen::Vector3d> &sp_points);

        std::vector<Eigen::Vector3d> spatial_points;
        std::vector<Eigen::Vector3d> image_points;
        Eigen::Vector3d normal;
    };

    std::vector<Eigen::Vector3d> get_points_in_cone(const std::vector<Eigen::Vector3d> &points);

    void recount_cone_params();

    static void intersect_points(std::list<Eigen::Vector3d> &points,
                                 const Eigen::Vector3d &plane_point, const Eigen::Vector3d &normal);

    std::vector<plane_convex_shape> shapes;

    double hor_angle_2;
    double ver_angle_2;
};

class img_map_observer : public map_observer {
public:
    img_map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                     int img_width, int img_height, int fov_hor);

    const float *render_to_img();

    ~img_map_observer() override;

private:
    void save_point(int x, int y, double d) override;

    float *image;
};

class pcl_map_observer : public map_observer {
public:
    pcl_map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                     int img_width, int img_height, int fov_hor);

    const pcl::PointCloud<pcl::PointXYZ> *render_to_pcl();

    ~pcl_map_observer() override = default;

private:
    void save_point(int x, int y, double d) override;

    pcl::PointCloud<pcl::PointXYZ> pcl;
};

class img_pcl_map_observer : public map_observer {
public:
    img_pcl_map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                         int img_width, int img_height, int fov_hor);

    void set_camera_pose(const Eigen::Affine3f &camera_pose) override;

    const float *render_to_img();

    const pcl::PointCloud<pcl::PointXYZ> *render_to_pcl();

    ~img_pcl_map_observer() override;

private:
    void save_point(int x, int y, double d) override;

    float *image;
    pcl::PointCloud<pcl::PointXYZ> pcl;

    bool actual_rendering_data;
};

class marked_map_observer : public map_observer {
public:
    marked_map_observer(const std::vector<std::vector<Eigen::Vector3d>> &marked_points,
                        const std::vector<std::vector<Eigen::Vector3d>> &shapes_,
                        int img_width, int img_height, int fov_hor);

    void render_to_marked_img_pts(std::vector<std::tuple<const Eigen::Vector3d *, int, int>> &marked_img_pts_);

    double get_focal_distance();

    ~marked_map_observer() override;

private:
    void save_point(int x, int y, double d) override;

    void operate_marked(int shape_idx) override;

    const std::vector<std::vector<Eigen::Vector3d>> &marked_points;
    std::vector<std::tuple<const Eigen::Vector3d *, int, int>> *marked_img_pts;
    std::pair<const Eigen::Vector3d *, float> *marks_image;
};