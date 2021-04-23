//
// Created by liuwei on 2021/4/21.
//

#ifndef STAIRS_DETECTION_CURRENT_SCENE_H
#define STAIRS_DETECTION_CURRENT_SCENE_H
#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/ransac.h>


#include "RGBD/plane.h"

class CurrentScene {

public:

    CurrentScene(){
    }

    ~CurrentScene(){

    }


    void applyVoxelFilter(float leaf_size,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);   // 体素滤波

    void getNormalsNeighbors(int neighbors);  // 邻域求法向量

    void regionGrowing();

    bool isPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr &region);

    void extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &points);


    void getCentroids();

    void getContours();


    void getPlaneCoeffs2Floor(Eigen::Affine3d c2f);


    void getCentroids2Floor(Eigen::Affine3d c2f);

    void classifyPlanes();







    // 参数
    pcl::PointCloud<pcl::PointXYZ>::Ptr fcloud;  // 滤波后的点云数据
    pcl::PointCloud<pcl::Normal>::Ptr normals; // 法向量点云
    pcl::search::Search<pcl::PointXYZ>::Ptr tree; // KdTree for fcloud
    std::vector<Plane> vPlanes; // 获取的平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_points; // Pointcloud of points not belonging to any plane for cluster extraction
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vObstacles; // Vector of non-planar pointcloud clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vObstacles2f; // Vector of non-planar pointcloud clusters at floor reference


};

#endif //STAIRS_DETECTION_CURRENT_SCENE_H

