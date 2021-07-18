#include <unordered_set>
#include <cstdlib>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>


// This is my custom RANSAC function
// for plane segmentation
template <typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
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
        PointT point1 = cloud->points[index1];
        PointT point2 = cloud->points[index2];
        PointT point3 = cloud->points[index3];
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