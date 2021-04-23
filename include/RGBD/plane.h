//
// Created by liuwei on 2021/4/21.
//

#ifndef STAIRS_DETECTION_PLANE_H
#define STAIRS_DETECTION_PLANE_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include <pcl/surface/concave_hull.h>
//#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>

class Plane {
public:
    Plane()	{

        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>() );

    }

    Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr region, Eigen::Vector4f plane_vector) {
        coeffs = plane_vector;
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>() );
        *cloud = *region;
        main_dir.setZero();
    }

    Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr region, Eigen::Vector4f plane_vector, pcl::PointXYZ centroid_in, pcl::PointXYZ centroid2f_in) {
        coeffs = plane_vector;
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>() );
        *cloud = *region;
        main_dir.setZero();
        centroid = centroid_in;
        centroid2f = centroid2f_in;
    }

    void getCentroid();   // 质心
    void getContour();    // 轮廓
    void getCoeffs2Floor(Eigen::Affine3d c2f);   // 将平面系数转换为楼层坐标
    void getCentroid2Floor(Eigen::Affine3d c2f); // 将质心转换为楼层坐标
    float getContourArea();

    void getPrincipalDirections();

    float getRectangleArea(Eigen::Matrix3f c2m);

    void getMeasurements();
    void getMeasurements(Eigen::Matrix3f c2m);
    void getMeasurements(Eigen::Affine3d & p2w);
    void getMeasurements(Eigen::Matrix3f c2m, pcl::PointCloud<pcl::PointXYZ>::Ptr custom_cloud);

    void getVertices();

    void getVertices(Eigen::Matrix3f c2m);



    int type; // 0 = floor, 1 = horizontal, 2 = lateral wall, 3 = frontal wall, 5 = anything else
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // Plane cloud平面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2f; // Plane cloud in floor coordinates
    pcl::PointXYZ centroid; // Plane centroid
    pcl::PointXYZ centroid2f; // Plane centroid in floor coordinates
    Eigen::Vector4f coeffs; // Plane coefficients
    Eigen::Vector4f coeffs2f; // Plane coefficients in floor coordinates
    pcl::PointCloud<pcl::PointXYZ>::Ptr contour; // Contour (concave hull)
    pcl::PointCloud<pcl::PointXYZ>::Ptr contour2f; // Contour in floor coordinates
    Eigen::Matrix3f main_dir; // 平面主要的方向
    float width, height, length; // Measurements of the bounding rectangle
    pcl::PointXYZ center; // Center of bounding rectangle
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices; // Vertices of the bounding rectangle

};

















#endif //STAIRS_DETECTION_PLANE_H
