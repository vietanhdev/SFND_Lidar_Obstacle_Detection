/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
      	int valueIndex = 0;
      	Node** currentNode = &root;
      	Node* newNode = new Node(point, id);
      
      	if (*currentNode == NULL) {
        	*currentNode = newNode;
          	return;
        }
      
      	while (true) {
          	if ((*currentNode)->point[valueIndex] > point[valueIndex]) { // Go to left
              	if ((*currentNode)->left != NULL) {
                	currentNode = &((*currentNode)->left);
                } else {
                  	(*currentNode)->left = newNode;
                  	break;
                }
            } else {
            	if ((*currentNode)->right != NULL) {
                	currentNode = &((*currentNode)->right);
                } else {
                  	(*currentNode)->right = newNode;
                  	break;
                }
            }
          	valueIndex = 1 - valueIndex;
        }
	}
  
  	void search(Node* currentNode, std::vector<int> &ids, const std::vector<float> &target, float distanceTol, int depth) {
      
      	if (currentNode == NULL) {
        	return;
        }
      
      	if (currentNode->point[0] - distanceTol <= target[0] && currentNode->point[0] + distanceTol >= target[0]
            && currentNode->point[1] - distanceTol <= target[1] && currentNode->point[1] + distanceTol >= target[1]) {
            float x_diff = currentNode->point[0] - target[0];
          	float y_diff = currentNode->point[1] - target[1];
        	float distance = sqrt(x_diff * x_diff + y_diff * y_diff);
          	if (distance < distanceTol) {
            	ids.push_back(currentNode->id);
            }
        }      
            
        int pid = depth % 2;
      	if (currentNode->point[pid] + distanceTol > target[pid]) {
        	search(currentNode->left, ids, target, distanceTol, depth + 1);
        }
      	if (currentNode->point[pid] - distanceTol < target[pid]) {
        	search(currentNode->right, ids, target, distanceTol, depth + 1);
        }
      
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
      	search(root, ids, target, distanceTol, 0);
		return ids;
	}
	

};




