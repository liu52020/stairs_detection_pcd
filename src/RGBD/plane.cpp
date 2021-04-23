//
// Created by liuwei on 2021/4/22.
//

#include "RGBD/plane.h"



void Plane::getCentroid() {
    Eigen::Vector4f vector_centroid;
    pcl::compute3DCentroid(*cloud,vector_centroid);
    centroid = pcl::PointXYZ (vector_centroid[0], vector_centroid[1], vector_centroid[2]);
}

void Plane::getContour() {
    // To obtain the point cloud, first it is necessary to project all points to a plane (using the coeffs from the Plane)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    pcl::ModelCoefficients::Ptr model_coeffs (new pcl::ModelCoefficients);
    model_coeffs->values.resize(4);

    model_coeffs->values[0] = coeffs(0);
    model_coeffs->values[1] = coeffs(1);
    model_coeffs->values[2] = coeffs(2);
    model_coeffs->values[3] = coeffs(3);
    proj.setModelCoefficients (model_coeffs);
    proj.filter (*cloud_projected);

    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setAlpha (0.1); // 0.1 works well in practice
    chull.setInputCloud (cloud_projected);
    contour.reset(new pcl::PointCloud<pcl::PointXYZ>);
    chull.reconstruct (*contour);
}

void Plane::getCoeffs2Floor(Eigen::Affine3d c2f) {
    coeffs2f = c2f.inverse().matrix().transpose().cast<float>()*coeffs;
}

void Plane::getCentroid2Floor(Eigen::Affine3d c2f) {
    centroid2f.x = static_cast<float> (c2f.matrix() (0, 0) * centroid.x + c2f.matrix() (0, 1) * centroid.y + c2f.matrix() (0, 2) * centroid.z + c2f.matrix() (0, 3));
    centroid2f.y = static_cast<float> (c2f.matrix() (1, 0) * centroid.x + c2f.matrix() (1, 1) * centroid.y + c2f.matrix() (1, 2) * centroid.z + c2f.matrix() (1, 3));
    centroid2f.z = static_cast<float> (c2f.matrix() (2, 0) * centroid.x + c2f.matrix() (2, 1) * centroid.y + c2f.matrix() (2, 2) * centroid.z + c2f.matrix() (2, 3));
}

float Plane::getContourArea() {
    if (contour == nullptr) {
        this->getContour();
    }
    float contour_area = pcl::calculatePolygonArea(*contour);

    return contour_area;
}

void Plane::getPrincipalDirections() {
    Eigen::Matrix3f covariance;
    Eigen::Vector4f eigen_centroid(centroid.x,centroid.y,centroid.z,1);
    pcl::computeCovarianceMatrixNormalized(*cloud,eigen_centroid,covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
    main_dir = eigen_solver.eigenvectors();
    main_dir.col(2) = main_dir.col(0).cross(main_dir.col(1));

    // Notice that column 0 has smaller eigenvalues, and thus direction perpendicular to the plane
}

void Plane::getMeasurements(Eigen::Matrix3f c2m) {

    Eigen::Matrix4f w2p(Eigen::Matrix4f::Identity());
    w2p.block<3,3>(0,0) = c2m.transpose();
    w2p.block<3,1>(0,3) = -1.f * (w2p.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());
    Eigen::Affine3d p2w = Eigen::Translation3d(centroid.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(c2m.cast<double>());

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, w2p);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,p2w.cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));
}

void Plane::getMeasurements(Eigen::Affine3d & c2m) {

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, c2m);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,c2m.inverse().cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));
}

void Plane::getMeasurements(Eigen::Matrix3f c2m, pcl::PointCloud<pcl::PointXYZ>::Ptr custom_cloud) {
    Eigen::Matrix4f w2p(Eigen::Matrix4f::Identity());
    w2p.block<3,3>(0,0) = c2m.transpose();
    w2p.block<3,1>(0,3) = -1.f * (w2p.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());
    Eigen::Affine3d p2w = Eigen::Translation3d(centroid.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(c2m.cast<double>());

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*custom_cloud, cPoints, w2p);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,p2w.cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));
}


float Plane::getRectangleArea(Eigen::Matrix3f c2m) {

    this->getMeasurements(c2m);

    float rectangle_area = width*length;

    return rectangle_area;
}

void Plane::getVertices() {
    vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // This function uses main_dir, thus, X is perpendicular to the plane and Z along the width

    Eigen::Vector3f pt1, pt2, pt3, pt4;
    pt1 = center.getVector3fMap()  + (width/2)*main_dir.col(2) + (length/2)*main_dir.col(1);
    pt2 = center.getVector3fMap()  - (width/2)*main_dir.col(2) + (length/2)*main_dir.col(1);
    pt3 = center.getVector3fMap()  - (width/2)*main_dir.col(2) - (length/2)*main_dir.col(1);
    pt4 = center.getVector3fMap()  + (width/2)*main_dir.col(2) - (length/2)*main_dir.col(1);

    vertices->push_back(pcl::PointXYZ(pt1(0),pt1(1),pt1(2)));
    vertices->push_back(pcl::PointXYZ(pt2(0),pt2(1),pt2(2)));
    vertices->push_back(pcl::PointXYZ(pt3(0),pt3(1),pt3(2)));
    vertices->push_back(pcl::PointXYZ(pt4(0),pt4(1),pt4(2)));
    vertices->push_back(center);
}

void Plane::getVertices(Eigen::Matrix3f c2m) {
    vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // This function uses dirs from, suposedly, Manhattan.
    // By our convention, X goes with the width and Z with the length

    Eigen::Vector3f pt1, pt2, pt3, pt4;
    pt1 = center.getVector3fMap() + (width/2)*c2m.col(0) + (length/2)*c2m.col(2);
    pt2 = center.getVector3fMap() - (width/2)*c2m.col(0) + (length/2)*c2m.col(2);
    pt3 = center.getVector3fMap() - (width/2)*c2m.col(0) - (length/2)*c2m.col(2);
    pt4 = center.getVector3fMap() + (width/2)*c2m.col(0) - (length/2)*c2m.col(2);

    vertices->push_back(pcl::PointXYZ(pt1(0),pt1(1),pt1(2)));
    vertices->push_back(pcl::PointXYZ(pt2(0),pt2(1),pt2(2)));
    vertices->push_back(pcl::PointXYZ(pt3(0),pt3(1),pt3(2)));
    vertices->push_back(pcl::PointXYZ(pt4(0),pt4(1),pt4(2)));
    vertices->push_back(center);
}