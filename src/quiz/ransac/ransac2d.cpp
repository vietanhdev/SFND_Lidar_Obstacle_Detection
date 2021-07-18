/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>

#include "../../processPointClouds.h"
#include "../../render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include <cstdlib>
#include <iostream>

#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++) {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--) {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd(
        "../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    size_t cloudSize = cloud->size();

    if (cloudSize < 2) {
        return inliersResult;
    }

    for (int i = 0; i < maxIterations; ++i) {
        std::unordered_set<int> inliers;
        int index1 = rand() % cloudSize;
        int index2 = index1;
        while (index2 == index1) {
            index2 = rand() % cloudSize;
        }
        pcl::PointXYZ point1 = cloud->points[index1];
        pcl::PointXYZ point2 = cloud->points[index2];
        float A = point1.y - point2.y;
        float B = point2.x - point1.x;
        float C = point1.x * point2.y - point2.x * point1.y;

        for (int j = 0; j < cloudSize; ++j) {
            float x = cloud->points[j].x;
            float y = cloud->points[j].y;
            float d = abs(A * x + B * y + C) / sqrt(A * A + B * B);
            if (d < distanceTol) {
                inliers.insert(j);
            }
        }

        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
            std::cout << "Max n inliers: " << inliers.size() << std::endl;
        }
    }

    return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    size_t cloudSize = cloud->size();

    if (cloudSize < 3) {
        return inliersResult;
    }

    for (int i = 0; i < maxIterations; ++i) {
        std::unordered_set<int> inliers;
        int index1 = rand() % cloudSize;
        int index2 = index1;
        int index3 = index1;
        while (index2 == index1) {
            index2 = rand() % cloudSize;
        }
        while (index3 == index1 || index3 == index2) {
            index3 = rand() % cloudSize;
        }
        pcl::PointXYZ point1 = cloud->points[index1];
        pcl::PointXYZ point2 = cloud->points[index2];
        pcl::PointXYZ point3 = cloud->points[index3];
        float x1 = point1.x;
        float y1 = point1.y;
        float z1 = point1.z;
        float x2 = point2.x;
        float y2 = point2.y;
        float z2 = point2.z;
        float x3 = point3.x;
        float y3 = point3.y;
        float z3 = point3.z;

        float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float D = -(A * x1 + B * y1 + C * z1);

        for (int j = 0; j < cloudSize; ++j) {
            float x = cloud->points[j].x;
            float y = cloud->points[j].y;
            float z = cloud->points[j].z;
            float d =
                abs(A * x + B * y + C * z + D) / sqrt(A * A + B * B + C * C);
            // std::cout << d << std::endl;
            if (d < distanceTol) {
                inliers.insert(j);
            }
        }

        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
            std::cout << "Max n inliers: " << inliers.size() << std::endl;
        }
    }

    return inliersResult;
}

int main() {
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
        new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    } else {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}
