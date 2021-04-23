//
// Created by liuwei on 2021/4/21.
//

#ifndef STAIRS_DETECTION_STAIR_CLASSES_H
#define STAIRS_DETECTION_STAIR_CLASSES_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include "RGBD/plane.h"

// 参数  可以之后再调
const float k_height_threshold = 0.04f;   // Around voxel grid value
const float k_length_threshold = 0.06f;   // Some stairs in the dataset have lengths around 23cm, thus I had to increase the threshold to be safe
const float k_height_min = 0.13f - k_height_threshold; // Min height is 13 cm
const float k_height_max = 0.185f + k_height_threshold; // Max height is 18.5 cm
//const float k_length_min = 0.28f - k_length_threshold; // Min length is 28 cm (no max length)
const float k_length_min = 0.10f - k_length_threshold;
const float k_sum_min = 0.54f - 2*k_height_threshold - k_length_threshold; // Sum of two risers and the length of one step must sum more than 54 cm
const float k_sum_max = 0.70f + 2*k_height_threshold + k_length_threshold; // Sum of two risers and the length of one step must sum less than 70 cm

int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr &neighbouring_cloud);
int neighbour_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, float radius);



class Stair {
public:

    Stair()	{
        step_height = 0;
        step_width = 0;
        step_length = 0;
    }
    ~Stair(){}


    Plane getBestStep ();

    void getStairDirFromPlane(Plane &plane);

    void getInitialStairVolume(Eigen::Matrix3f manhattan_dirs, Eigen::Matrix3f standalone_dirs, bool has_manhattan);


    void getInitialStepVertices();


    void getInitialStepVertices(Eigen::Matrix3f & dir);


    void getExactStepVertices();


    void modelStaircase(Eigen::Matrix3f main_dir, bool has_manhattan);

    bool validateStaircase();

    float step_width;
    float step_length;
    float step_height;

    std::string type; // up and down

    std::vector<Plane> vPlanes; // Step candidates given by the detection process 检测过程给出的步骤候选  检测结果候选
    std::vector<Plane> vLevels; // Step candidates ordered in "levels" regarding their distance to the floor in steps
    std::vector<Plane> vOrientedLevels;
    std::vector<Plane> vRisers; //
    Eigen::Matrix3f stair_dir; //
    Eigen::Affine3d i2s; //
    Eigen::Affine3d s2i; //
    pcl::PointXYZ initial_point; //
};








#endif //STAIRS_DETECTION_STAIR_CLASSES_H
