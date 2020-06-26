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

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	/* RANSAC Algorithm
	 * Picks 3 points at random to form a plane, and adds to inliers all points where d < distanceTol.
	 * Repeats maxIterations # times, and returns ground plane with the most inliers.
	 */

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand() % cloud->points.size());
		}

		auto it = inliers.begin();
		float x1 = cloud->points[*it].x;
		float y1 = cloud->points[*it].y;
		float z1 = cloud->points[*it].z;
		++it;
		float x2 = cloud->points[*it].x;
		float y2 = cloud->points[*it].y;
		float z2 = cloud->points[*it].z;
		++it;
		float x3 = cloud->points[*it].x;
		float y3 = cloud->points[*it].y;
		float z3 = cloud->points[*it].z;

		float a = ((y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1));
		float b = ((z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1));
		float c = ((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1));
		float d = -(a*x1 + b*y1 + c*z1);

		for (int i = 0; i < cloud->points.size(); ++i) {
			if (inliers.count(i))  continue;

			pcl::PointXYZ point = cloud->points[i];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float dist = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);
			if (dist <= distanceTol) {
				inliers.insert(i);
			}
		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}

	return inliersResult;
}
