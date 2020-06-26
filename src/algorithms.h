#ifndef KDTREE3D_H
#define KDTREE3D_H

#include <unordered_set>
#include <vector>

#include <pcl/visualization/pcl_visualizer.h>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	Node* root;

	KdTree() : root(nullptr) {}

	void insert(Node** currNode, int depth, std::vector<float> point, int id) {
		if (*currNode == nullptr) {
			*currNode = new Node(point, id);
		}
		else {
			int dim = depth % 3;

			if (point[dim] < (*currNode)->point[dim]) {
				insert(&(*currNode)->left, depth + 1, point, id);
			} else {
				insert(&(*currNode)->right, depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insert(&root, 0, point, id);
	}

	void search(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int>& ids) {
		if (node) {
			if (fabs(node->point[0] - target[0]) <= distanceTol &&
				fabs(node->point[1] - target[1]) <= distanceTol &&
				fabs(node->point[2] - target[2]) <= distanceTol)
			{
				if (EuclideanDistance(node->point, target) <= distanceTol) {
					ids.emplace_back(node->id);
				}
			}

			int dim = depth % 3;
			if (target[dim] - node->point[dim] < distanceTol) {
				search(target, distanceTol, node->left, depth + 1, ids);
			}
			if (node->point[dim] - target[dim] < distanceTol) {
				search(target, distanceTol, node->right, depth + 1, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(target, distanceTol, root, 0, ids);
		return ids;
	}
	
	float EuclideanDistance(std::vector<float> pt1, std::vector<float> pt2) {
		return sqrt(pow(pt2[0] - pt1[0], 2) 
				  + pow(pt2[1] - pt1[1], 2) 
				  + pow(pt2[2] - pt1[2], 2)); 
	}
};


// Euclidean Clustering  - used to group object point clouds
void proximity(const std::vector<std::vector<float>>& points, int point, KdTree* tree,
			   std::vector<int>& cluster, std::unordered_set<int>& explored, float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
											   KdTree* tree, float distanceTol);


// 3D RANSAC Algorithm - used to segment out ground plane
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
									int maxIterations, float distanceTol);

#endif