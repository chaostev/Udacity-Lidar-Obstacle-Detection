#include "algorithms.h"

void proximity(const std::vector<std::vector<float>>& points, int point, KdTree* tree,
			   std::vector<int>& cluster, std::unordered_set<int>& explored, float distanceTol) {
	if (explored.count(point))  return;
	explored.insert(point);

	cluster.emplace_back(point);
	std::vector<int> nearby = tree->search(points[point], distanceTol);
	for (int p : nearby) {
		proximity(points, p, tree, cluster, explored, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
											   KdTree* tree, float distanceTol) {
	std::vector<std::vector<int>> clusters;
	std::unordered_set<int> explored;
 
	for (int i = 0; i < points.size(); ++i) {
		if (explored.count(i))  continue;

		std::vector<int> cluster;
		proximity(points, i, tree, cluster, explored, distanceTol);
		clusters.emplace_back(cluster);
	}

	return clusters;
}
