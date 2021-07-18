#ifndef KD_TREE_CLUSTERING_H_
#define KD_TREE_CLUSTERING_H_

#include <pcl/common/common.h>

#include <cmath>
#include <vector>
#include <queue>

// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
    Node* root;
    int k = 3; // Hardcode for now

    KdTree() : root(NULL) {}

    void insert(std::vector<float> point, int id) {
        int pid = 0;
        Node** currentNode = &root;
        Node* newNode = new Node(point, id);

        if (*currentNode == NULL) {
            *currentNode = newNode;
            return;
        }

        while (true) {
            if ((*currentNode)->point[pid] > point[pid]) {  // Go to left
                if ((*currentNode)->left != NULL) {
                    currentNode = &((*currentNode)->left);
                } else {
                    (*currentNode)->left = newNode;
                    break;
                }
            } else {  // Go to right
                if ((*currentNode)->right != NULL) {
                    currentNode = &((*currentNode)->right);
                } else {
                    (*currentNode)->right = newNode;
                    break;
                }
            }
            pid = (pid + 1) % k;
        }
    }

    void search(Node* currentNode, std::vector<int>& ids,
                const std::vector<float>& target, float distanceTol,
                int depth) {
        if (currentNode == NULL) {
            return;
        }
        if (currentNode->point[0] - distanceTol <= target[0] &&
            currentNode->point[0] + distanceTol >= target[0] &&
            currentNode->point[1] - distanceTol <= target[1] &&
            currentNode->point[1] + distanceTol >= target[1] &&
            currentNode->point[2] - distanceTol <= target[2] &&
            currentNode->point[2] + distanceTol >= target[2]) {
            float x_diff = currentNode->point[0] - target[0];
            float y_diff = currentNode->point[1] - target[1];
            float z_diff = currentNode->point[2] - target[2];
            float distance = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
            if (distance < distanceTol) {
                ids.push_back(currentNode->id);
            }
        }

        int pid = depth % k;
        if (currentNode->point[pid] + distanceTol > target[pid]) {
            search(currentNode->left, ids, target, distanceTol, depth + 1);
        }
        if (currentNode->point[pid] - distanceTol < target[pid]) {
            search(currentNode->right, ids, target, distanceTol, depth + 1);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol) {
        std::vector<int> ids;
        search(root, ids, target, distanceTol, 0);
        return ids;
    }
};

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points, KdTree* tree,
    float distanceTol, float minSize, float maxSize) {
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed;
    processed.resize(points.size(), false);
    for (int i = 0; i < points.size(); ++i) {
        if (processed[i]) continue;
        std::queue<int> processing({i});
        std::vector<int> cluster;
        while (!processing.empty()) {
            int current = processing.front();
            processing.pop();
            if (!processed[current]) {
                cluster.push_back(current);
                processed[current] = true;
                std::vector<int> near_ids = tree->search(points[current], distanceTol);
                for (int j = 0; j < near_ids.size(); ++j) {
                    if (!processed[near_ids[j]]) {
                        processing.push(near_ids[j]);
                    }
                }
            }
        }

        if (cluster.size() >= minSize && cluster.size() <= maxSize) {
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol,
    float minSize, float maxSize) {
    // Prepare points
    std::vector<std::vector<float>> points;
    for (auto point : cloud->points) {
        std::vector<float> pointVec({point.x, point.y, point.z});
        points.push_back(pointVec);
    }

    // Prepare KDTree
    KdTree* tree = new KdTree;
    for (int i = 0; i < points.size(); i++) tree->insert(points[i], i);

    // Do clustering
    std::vector<std::vector<int>> clustersIndices =
        euclideanCluster(points, tree, distanceTol, minSize, maxSize);

    // Convert clusters indices to point clouds
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for (int i = 0; i < clustersIndices.size(); ++i) {
        typename pcl::PointCloud<PointT>::Ptr cluster(
            new pcl::PointCloud<PointT>);
        for (const auto& idx : clustersIndices[i])
            cluster->push_back((*cloud)[idx]);
        clusters.push_back(cluster);
    }

    return clusters;
}

#endif
