//
// Created by liuwei on 2021/4/21.
//

#include "visualizer.h"

void Viewer::drawCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){

    pcl::visualization::CloudViewer cloud_viewer("123");
    cloud_viewer.showCloud(cloud);
    while (!cloud_viewer.wasStopped())
    {
    }

}
