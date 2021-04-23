//
// Created by liuwei on 2021/4/21.
//

#include "RGBD/current_scene.h"

void CurrentScene::applyVoxelFilter(float leaf_size,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    //  体素滤波的结果放入了fcloud中
    fcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*fcloud);
}

void CurrentScene::getNormalsNeighbors(int neighbors) {

    normals.reset(new pcl::PointCloud<pcl::Normal>);
    //tree =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);  // boost 改为std
    //pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (fcloud);
    normal_estimator.setKSearch (neighbors);
    normal_estimator.useSensorOriginAsViewPoint();

    normal_estimator.compute (*normals);

}

bool sortIndicesBySize(const pcl::PointIndices &lhs, const pcl::PointIndices &rhs) {
    return lhs.indices.size() > rhs.indices.size(); // Large planes to small planes
}


void CurrentScene::regionGrowing() {   //
    // Configure region growing:
    const int k_min_cluster_size = 30;
    const int k_num_neighbors = 16;
    const float k_angle_threshold = 6.0;
    const float k_curvature_threshold = 0.5;

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (k_min_cluster_size);
    reg.setSmoothModeFlag(true);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (k_num_neighbors);//8
    reg.setInputCloud (fcloud); //fcloud
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (pcl::deg2rad(k_angle_threshold));
    reg.setCurvatureThreshold (k_curvature_threshold);//0.5
    std::vector <pcl::PointIndices> regions;
    reg.extract (regions);

    pcl::PointIndices::Ptr total_indices (new pcl::PointIndices);

    sort(regions.begin(), regions.end(), sortIndicesBySize);

    remaining_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    *remaining_points = *fcloud;

    for (size_t Q=0; Q < regions.size(); Q++) {
        if (regions[Q].indices.size() > k_min_cluster_size) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::ExtractIndices<pcl::PointXYZ> ex;
            ex.setInputCloud (fcloud);
            pcl::PointIndices::Ptr indices_ (new pcl::PointIndices);
            *indices_ = regions[Q];
            ex.setIndices (indices_);
            ex.setNegative (false);
            ex.filter (*cloud_temp);

            // Call to extract Plane (if it is) and add it to vPlanes, or add cloud to vObstacles
            this->isPlane(cloud_temp);

            total_indices->indices.insert(total_indices->indices.end(), indices_->indices.begin(), indices_->indices.end());
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud (remaining_points);
    ex.setIndices (total_indices);
    ex.setNegative (true);
    ex.filter (*remaining_points);

}

bool CurrentScene::isPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &region) {
    bool is_plane = false;
    // Create the segmentation object for the planar model and set all the parameters
    const int k_max_iter = 100; // to iterate the RANSAC
    const double k_dist_threshold = 0.04; // To accept points as inliers in the RANSAC
    const float k_min_ratio = 0.80f; // Percentage of points of the cloud belonging to the plane

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (k_max_iter);
    seg.setDistanceThreshold (k_dist_threshold);

    // Segment the largest planar component from the remaining cloud 从剩余的云中分割最大的平面组件
    seg.setInputCloud (region);
    seg.segment (*inliers, *coefficients);

    float n_points_total = region->points.size();
    float n_points_plane = inliers->indices.size();
    float ratio = n_points_plane/n_points_total;

    if (ratio > k_min_ratio) {
        is_plane = true;

        Eigen::Vector4f plane_vector(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
        if (plane_vector[3] < 0) {plane_vector = -plane_vector;}

        Plane plane(region, plane_vector);

        vPlanes.push_back(plane);
    }
    else
        vObstacles.push_back(region);

    return is_plane;
}

void CurrentScene::extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) {
    const int k_min_cluster_size = 50;
    const double k_cluster_tolerance = 0.05;

    if (points->points.size() > k_min_cluster_size) {
        tree->setInputCloud (points);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (k_cluster_tolerance);
        ec.setMinClusterSize (k_min_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (points);
        ec.extract (cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_aux (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster_aux->points.push_back (points->points[size_t(*pit)]);
            cloud_cluster_aux->width = uint32_t(cloud_cluster_aux->points.size());
            cloud_cluster_aux->height = 1;
            cloud_cluster_aux->is_dense = true;

            vObstacles.push_back(cloud_cluster_aux);
        }
    }
}


void CurrentScene::getCentroids() {
    for (size_t Q=0; Q<vPlanes.size(); Q++)
        vPlanes[Q].getCentroid();
}

void CurrentScene::getContours() {
    for (size_t Q=0; Q<vPlanes.size(); Q++) {
        vPlanes[Q].getContour();
    }
}

void CurrentScene::getPlaneCoeffs2Floor(Eigen::Affine3d c2f) {
    for (size_t Q=0; Q<vPlanes.size();Q++)
        vPlanes[Q].getCoeffs2Floor(c2f);
}

void CurrentScene::getCentroids2Floor(Eigen::Affine3d c2f) {
    for (size_t Q=0; Q<vPlanes.size();Q++) {
        vPlanes[Q].getCentroid2Floor(c2f);
    }
}



void CurrentScene::classifyPlanes() {
    // 分类的阈值
    const float k_angle_threshold = 15.0f;
    const float k_dist_floor_threshold = 0.08f;

    for (size_t Q=0; Q<vPlanes.size();Q++) {
        float A = vPlanes[Q].coeffs2f[0];   //
        float B = vPlanes[Q].coeffs2f[1];
        float C = vPlanes[Q].coeffs2f[2];

        float dot = vPlanes[Q].coeffs2f.head<3>().dot(Eigen::Vector3f::UnitY());
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) ); // to avoid NaNs
        float angle = pcl::rad2deg(acos(dot));   // 弧度转角度

        if (fabs(A)>fabs(B) and fabs(A)>fabs(C)) {         // If component in X is higher (lateral plane)
            if (fabs(90-angle)<k_angle_threshold) // if angle is close to perpendicular to floor normal
                vPlanes[Q].type = 2;
            else
                vPlanes[Q].type = 5;
        }
        else if (fabs(C)>fabs(A) and fabs(C)>fabs(B)) {        // If component in Z is higher (frontal plane)
            if (fabs(90-angle)<k_angle_threshold) // if angle is close to perpendicular to floor normal
                vPlanes[Q].type = 3;
            else
                vPlanes[Q].type = 5;
        }
        else if (fabs(B)>fabs(A) and fabs(B)>fabs(C)) {         // If component in Y is higher (horizontal plane)
            if (angle<k_angle_threshold){ // || fabs(angle-180) < 15) { // if angle is close to floor normal it is horizontal
                if (fabs(vPlanes[Q].centroid2f.y) <= k_dist_floor_threshold) // If close to floor, then it is floor
                    vPlanes[Q].type = 0;
                else
                    vPlanes[Q].type = 1;
            }
            else
                vPlanes[Q].type = 5;
        }
    }
}