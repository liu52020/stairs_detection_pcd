//
// Created by liuwei on 2021/4/21.
//

#ifndef STAIRS_DETECTION_GLOBAL_SCENE_H
#define STAIRS_DETECTION_GLOBAL_SCENE_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/angles.h>
#include <pcl/common/common_headers.h>

#include "RGBD/plane.h"
#include "RGBD/current_scene.h"

class GlobalScene {

public:

    GlobalScene() {

        initial_floor_ = false;

    }



    void findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    bool subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal);

    void computeCamera2FloorMatrix (Eigen::Vector4f floor_normal);

    void findFloorFast(std::vector<Plane> vPlanes);


    bool initial_floor_; //
    Eigen::Vector4f floor_normal_; // 楼层平面系数
    Eigen::Affine3d c2f; // transformation matrix camera to floor 变换矩阵摄像机到地面
    Eigen::Affine3d f2c; // transformation matrix floor to camera
    float person_height_; // estimated height of the camera to floor 摄像机到地面的估计高度
    bool new_floor_; // 如果在当前场景中找到地板，则为true
    bool has_manhattan_; // true if manhattan directions have been found 如果找到曼哈顿方向，则为真
    Eigen::Matrix3f main_dir; // Principal directions (Manhattan directions) on the global scene 全球主要方向（曼哈顿方向）

};

#endif //STAIRS_DETECTION_GLOBAL_SCENE_H
