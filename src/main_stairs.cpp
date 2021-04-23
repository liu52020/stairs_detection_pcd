//
// Created by liuwei on 2021/4/21.
//

#include <math.h>
#include <iostream>
#include <signal.h>
#include <time.h>
#include <dirent.h> // To read directory
#include<string>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/PCLHeader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>

#include "visualizer.h"
#include "stair/current_scene_stair.h"
#include "stair/global_scene_stair.h"
#include "stair/stair_classes.h"
using namespace std;

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>


// 内参
const float factor = 1000;
const float f = 613;  // 焦距
const float cx = 327;
const float cy = 225;


class MainLoop{
public:
    MainLoop(){

        depth_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);


    }

    ~MainLoop(){

    }

    // 输入深度图  转点云
    void startPCD() {

        int index = 0 ;

        // cloud
        pcl::io::loadPCDFile("/home/liuwei/data/Tang_dataset_PCD/Positive_PCD/82_Cloud_XYZRGB.pcd", *cloud);
        // depth_cloud
        cv::Mat depth;
        const char filename[] = "/home/liuwei/data/stair/depth/250.png";
        //“2”拿深度
        depth = cv::imread(filename, 2);
        // 遍历深度图 // 用灰度图去生成点云
        for (int m = 0; m < depth.rows; m++) {
            for (int n = 0; n < depth.cols; n++) {
                ushort d = depth.ptr<ushort>(m)[n];
                pcl::PointXYZ p;
                p.z = double(d) / factor;
                p.x = (n - cx) * p.z / f;
                p.y = (m - cy) * p.z / f;
                // 把p加入到点云中
                depth_cloud->points.push_back(p);
            } // cloud 是一个三维的点云数据

        }

        while(true) {
            if (depth_cloud->points.size() > 0) {
                this->execute();
            }

            index++;
            if(index>1){   // 只操作一次   后面用可视化进行判断
                break;
            }
//            if (viewer.cloud_viewer_.wasStopped())
//                break;
        }
    }
    // 执行函数
    void execute() {

        // 在当前场景去处理点云数据
        CurrentSceneStair scene;    // 当前的楼梯场景 继承了场景类  创建对象
        //scene.applyVoxelFilter(0.04f, cloud); //  增加一个体素网格  输出：scene.fcloud
        scene.applyVoxelFilter(0.04f, depth_cloud); //  增加一个体素网格  输出：scene.fcloud
        if (!gscene.initial_floor_){  // 判断地板是否存在
            // 初始化  假设存在  直到找到地板
            // 找到地板之后 initial_floor_=true  开始对点云数据进行分析

            gscene.findFloor(scene.fcloud);  // 将经过体素滤波的点云放入
            // 找到地板之后 initial_floor_变为1
            gscene.computeCamera2FloorMatrix(gscene.floor_normal_);

        }
        else{  // 找到地板之后   initial_floor_ 变为true之后
            // 开始对点云数据进行操作
            //scene.getNormalsNeighbors(16);
//            std::cout<<"程序找到地板后进行操作！"<<endl;
                // 计算法向量
            std::cout<<"使用区域生长"<<endl;
            scene.getNormalsNeighbors(16);   //

            scene.regionGrowing(); // 区域生长

            //scene.extractClusters(scene.remaining_points);  // 欧氏聚类提取   也是一个滤波

            //gscene.findFloorFast(scene.vPlanes); // 查找并更新地板位置

            gscene.findFloorFast(scene.vPlanes);

            if (gscene.new_floor_)  // 通过地面发现 计算相机到地板矩阵
                gscene.computeCamera2FloorMatrix(gscene.floor_normal_);

            // 如果没找到  还是楼梯
            // 获取楼梯的点云数据   获得质心，轮廓和平面系数到楼层参考
            scene.getCentroids();
            scene.getContours();
            scene.getPlaneCoeffs2Floor(gscene.c2f);
            scene.getCentroids2Floor(gscene.c2f);

            // 分类
            scene.classifyPlanes();


            std::cout<<"平面的类型有："<<endl;
            for(int i=0;i<scene.vPlanes.size();++i){
                std::cout<<scene.vPlanes[i].type
                        <<" "<<scene.vPlanes[i].centroid2f.x
                        <<" "<<scene.vPlanes[i].centroid2f.y
                        <<" "<<scene.vPlanes[i].centroid2f.z<<std::endl;
            }

//            std::cout<<"使用区域生长"<<endl;

//            std::cout<<"区域生长之后点云数据剩余大小："<<scene.remaining_points->points.size()<<std::endl;
//            std::cout<<"获取的平面数："<<scene.vPlanes.size()<<endl;
                //std::cout<<scene.vObstacles.size()<<endl;
            //std::cout<<"是否找到新的地板："<<gscene.new_floor_<<endl;
        }


        // 可视化部分
           viewer.drawCloud(depth_cloud);  // 显示原始点云
//          std::cout<<"原点云数据的大小："<<cloud->points.size()<<std::endl;
          // std::cout<<"是否找到地板："<<gscene.initial_floor_<<endl;
//        std::cout<<"体素滤波后点云数据的大小："<<scene.fcloud->points.size()<<std::endl;
//        std::cout<<"摄像头与地面的距离："<<gscene.person_height_<<endl;
//        std::cout<<"是否找到地板："<<gscene.initial_floor_<<endl;
         // viewer.drawCloud(scene.remaining_points);  // 显示滤波点云
        viewer.drawCloud(scene.fcloud);  // 显示滤波点云

// 阶梯检测
        if (scene.detectStairs()) {
            // Ascending staircase  //

            if (scene.getLevelsFromCandidates(scene.upstair,gscene.c2f)) { // Sort planes in levels
                scene.upstair.modelStaircase(gscene.main_dir, gscene.has_manhattan_); // Perform the modeling
                if (scene.upstair.validateStaircase()) { // Validate measurements
                    std::cout << "--- ASCENDING STAIRCASE ---\n" <<
                              "- Steps: " << scene.upstair.vLevels.size()-1 << std::endl <<
                              "- Measurements: " <<
                              scene.upstair.step_width << "m of width, " <<
                              scene.upstair.step_height << "m of height, " <<
                              scene.upstair.step_length << "m of length. " << std::endl <<
                              "- Pose (stair axis w.r.t. camera):\n" << scene.upstair.s2i.matrix() << std::endl << std::endl;
                }

            }

            // Descending staircase
            if (scene.getLevelsFromCandidates(scene.downstair,gscene.c2f)) { // Sort planes in levels
                scene.downstair.modelStaircase(gscene.main_dir, gscene.has_manhattan_); // Perform the modeling
                if (scene.downstair.validateStaircase()) { // Validate measurements
                    std::cout << "--- DESCENDING STAIRCASE ---\n" <<
                              "- Steps: " << scene.downstair.vLevels.size()-1 << std::endl <<
                              "- Measurements: " <<
                              scene.downstair.step_width << "m of width, " <<
                              scene.downstair.step_height << "m of height, " <<
                              scene.downstair.step_length << "m of length. " << std::endl <<
                              "- Pose (stair axis w.r.t. camera):\n" << scene.downstair.s2i.matrix() << std::endl << std::endl;

            }


            }

        }


    }


    // 参数

    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    // 类和对象

    Viewer viewer;
    GlobalSceneStair gscene; // 全局场景

};



int main(){

    MainLoop app;
    app.startPCD();

    // 可视化


}