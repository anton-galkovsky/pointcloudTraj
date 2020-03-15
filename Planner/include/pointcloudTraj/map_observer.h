#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class map_observer {
public:
//    class data_saver {
//    public:
//        virtual void init_storage() = 0;
//
//        virtual void save_point(int x, int y, double d) = 0;
//    };
//
//    class img_saver : data_saver {
//    public:
//        explicit img_saver(int width_, int height_);
//
//        void init_storage() override;
//
//        void save_point(int x, int y, double d) override;
//
//        float *get_image();
//
//        ~img_saver();
//
//    private:
//        int width;
//        int height;
//        float *image;
//    };
//
//    class pcl_saver : data_saver {
//    public:
//        pcl_saver();
//
//        void init_storage() override;
//
//        void save_point(int x, int y, double d) override;
//
//        pcl::PointCloud<pcl::PointXYZ> *get_pcl();
//
//    private:
//        pcl::PointCloud<pcl::PointXYZ> pcl;
//    };

    virtual void set_camera_pose(const Eigen::Affine3f &camera_pose);

protected:
    map_observer(const std::vector<std::vector<Eigen::Vector3d>> &shapes_, int img_width, int img_height, int fov_hor);

    void render();

    virtual void save_point(int x, int y, double d) = 0;

    virtual ~map_observer() = default;

    Eigen::Vector3d camera_axis_x;
    Eigen::Vector3d camera_axis_y;
    Eigen::Vector3d camera_axis_z;
    Eigen::Vector3d camera_translation;

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

    Eigen::Vector3d cone_normals[4];

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