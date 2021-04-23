//
// Created by liuwei on 2021/4/21.
//

#ifndef STAIRS_DETECTION_VISUALIZER_H
#define STAIRS_DETECTION_VISUALIZER_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/cloud_viewer.h>


class Viewer {
public:

    Viewer(){

    }

    void drawCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);


    pcl::visualization::PCLVisualizer cloud_viewer_;

};
#endif //STAIRS_DETECTION_VISUALIZER_H
