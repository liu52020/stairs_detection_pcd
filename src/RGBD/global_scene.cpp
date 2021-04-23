//
// Created by liuwei on 2021/4/21.
//

#include "RGBD/global_scene.h"

void GlobalScene::findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    float z_dist = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    const int k_min_cluster_size = 500;
    const float k_max_z_dist = 5.0;
    const float k_z_dist_increase = 0.1f;

    while (cloud_filtered->points.size() < k_min_cluster_size) {
        // 先直接进入  达到一定的条件之后再退出  500
        //
        pcl::PassThrough<pcl::PointXYZ> pass;   // 直通滤波
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, z_dist);
        //pass.setFilterLimitsNegative(false);  这个可以去掉 好像是保存
        pass.filter(*cloud_filtered);

        if (z_dist > k_max_z_dist)
            break;
        else
            z_dist += k_z_dist_increase;

    }

    // 一旦我们有了一个过滤过的云，查找多达n个平面，然后尝试评估它们是否满足有效地板的条件
    const int k_n_planes_try = 5;
    int n_planes_try = k_n_planes_try;

    while ((!initial_floor_) and (n_planes_try >= 0)) {
        if (n_planes_try == 0) {
            z_dist += k_z_dist_increase; // 如果没有一个n_平面生成有效的初始_楼层，请增加z_距离
            if (z_dist > k_max_z_dist)
                break;
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.0, z_dist);
            pass.filter(*cloud_filtered);

            n_planes_try = k_n_planes_try;
        }
        else {
            initial_floor_ = this->subtractInitialPlane(cloud_filtered, floor_normal_);
            n_planes_try -= 1;
        }
    }
}

bool GlobalScene::subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal) {

    bool initial_plane = false; // 如果可用的平面找到了  地板

    const int k_min_plane_size = 300;
    const float k_min_percentage_size = 0.30f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());
    *cloud_filtered=*cloud;  // 直接赋值

    if (cloud_filtered->points.size()>k_min_plane_size){ // Three points at least to find a plane
        // Perform RANSAC segmentation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02); // aprox. Voxel size / 2

        // 最大的部分
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);

        // Either the plane is very large (at least k_min_plane_size points) or less size but occupying large part of the scene (high percentage of the initial cloud)
        if (inliers->indices.size() > k_min_plane_size ||
            (inliers->indices.size() > float(k_min_plane_size)*k_min_percentage_size && float(inliers->indices.size())/float(cloud_filtered->points.size()) > k_min_percentage_size)) {

            // Plane coefficients   Ax + By + Cz + D =0
            float A=(coefficients->values[0]);
            float B=(coefficients->values[1]);
            float C=(coefficients->values[2]);
            float D=(coefficients->values[3]);

            if (D < 0) {A = -A; B = -B; C = -C; D = -D;} // To get the plane normal looking towards the origin

            Eigen::Vector3f v_plane(A,B,C);
            Eigen::Vector3f v_floor(0.0f, -sin(float(M_PI_4)), -sin(float(M_PI_4))); // Estimated position of the floor normal considering the camera is looking downwards, circa 45 degrees

            float dot = v_plane.dot(v_floor);
            dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) ); // to avoid NaNs
            float angle = pcl::rad2deg(acos(dot));

            // Parameters to consider valid floor
            const float k_angle_threshold = 30; // Valid threshold around estimated floor vector
            const float k_min_D = 1.0f; // Minimum D value (i.e. minimum distance of the floor to the camera allowed)
            const float k_max_D = 1.8f; // Maximum D value (i.e. maximum distance of the floor to the camera allowed)

            if (angle < k_angle_threshold && fabs(D) > k_min_D && fabs(D) < k_max_D) {
                std::cout << std::endl <<"-- Floor found --" << std::endl;
                std::cout << "Plane coefficients -> A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
                std::cout << "Angle with respect to the estimated vertical normal = " << angle << std::endl;
                std::cout << "Plane contains " << inliers->indices.size() << " points" << std::endl << std::endl;

                normal = Eigen::Vector4f(A,B,C,D);

                initial_plane = true;
            }
        }
        else {
            initial_plane = false;
        }

        // 如果没有平面 返回剩余的点云数据
        if (!initial_plane) {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cloud);

//            std::cout<<"剩余数据："<<cloud_filtered->points.size()<<std::endl;
        }

    }

    return (initial_plane);
}

void GlobalScene::computeCamera2FloorMatrix (Eigen::Vector4f floor_normal){
    // Transformation from Camera to Floor reference frame (c2f) is computed knowing one of the axis (the floor normal, corresponding to Y in our convention)
    //从相机到地面参考帧（c2f）的转换是在已知一条轴（地面法线，在我们的约定中对应于Y）的情况下计算的
    Eigen::Matrix3f R;  // 矩阵
    Eigen::Vector3f t = Eigen::Vector3f::Zero();

    float a,b,c,d;
    c = sqrt(1/(1+floor_normal(2)*floor_normal(2)/(floor_normal(1)*floor_normal(1))));
    d = -c*floor_normal(2)/floor_normal(1);
    b = 1/(floor_normal(0)+(c*floor_normal(1)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0)-(d*floor_normal(2)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0));
    a = b*(c*floor_normal(1)-d*floor_normal(2))/floor_normal(0);

    R << a, -b*c, b*d, b, a*c, -a*d, 0, d, c;
    t(1) = -a*floor_normal(3);

    c2f = ( Eigen::Translation3d (t.cast<double>()) * Eigen::AngleAxisd (R.cast<double>()));
    f2c = c2f.inverse();
    person_height_ = float(fabs(c2f(1,3)));

}

void GlobalScene::findFloorFast(std::vector<Plane> vPlanes) {
    // Instead of performing a RANSAC each time, we can use the planes from region growing (computed in current scene)

    // Threshold to consider vPlanes[Q] a new valid floor
    const float k_angle_threshold = 15; // degrees with respect to previous floor_normal
    const float k_D_threshold = 0.08f; // Distance to camera with respect to previous D 相对于上一个D到摄像机的距离

    new_floor_ = false;
    for (size_t Q=0; Q<vPlanes.size();Q++)	{

        float dot = vPlanes[Q].coeffs.head<3>().dot(floor_normal_.head<3>());
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) );
        float angle = pcl::rad2deg(acos(dot));

        if ((angle < k_angle_threshold) and (fabs(fabs(vPlanes[Q].coeffs[3])-fabs(floor_normal_[3]))<=k_D_threshold)) {
            floor_normal_ = vPlanes[Q].coeffs;
            new_floor_ = true;
            break;
        }
    }
}