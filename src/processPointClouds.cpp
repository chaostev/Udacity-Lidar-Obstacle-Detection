// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Downsample points to voxels of size filterRes
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered( new pcl::PointCloud<PointT> );
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Crop point cloud to relevant region of points
    typename pcl::PointCloud<PointT>::Ptr cloudRegion( new pcl::PointCloud<PointT> );
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // Filter out roof points
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.setNegative(true);
    roof.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr plane( new pcl::PointCloud<PointT> );
    typename pcl::PointCloud<PointT>::Ptr obstacles( new pcl::PointCloud<PointT> );

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);
    extract.setNegative(true);
    extract.filter(*obstacles);

    return std::make_pair(obstacles, plane);
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

    // My 3D RANSAC Plane Segmentation
    std::unordered_set<int> inlierIds = RansacPlane(cloud, maxIterations, distanceThreshold);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (auto it = inlierIds.begin(); it != inlierIds.end(); ++it) {
        inliers->indices.emplace_back(*it);
    }

    /*
    // PCL methods
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients { new pcl::ModelCoefficients};

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given data\n";
    }
    */

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // My 3D KDTree Method
    // Note: not a great solution, since it will use O(n) addtional memory,
    //  but faster than adapting all the function signatures
    // TODO: adapt kdtree signatures to use pcl::PointCloud<PointT>::Ptr cloud
    std::vector<std::vector<float>> points;
    for (auto pt : cloud->points) {
        std::vector<float> vPt = {pt.x, pt.y, pt.z};
        points.emplace_back(std::move(vPt));
    }

    KdTree* tree = new KdTree;
    for (int i = 0; i < points.size(); ++i) {
        tree->insert(points[i], i);
    }
    
  	std::vector<std::vector<int>> processedClusters = euclideanCluster(points, tree, clusterTolerance);
    
    for (auto clusterIndices : processedClusters) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster( new pcl::PointCloud<PointT> );

        for (int i : clusterIndices) {
            cloudCluster->points.emplace_back(cloud->points[i]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        if (cloudCluster->points.size() >= minSize && cloudCluster->points.size() <= maxSize) {
            clusters.emplace_back(cloudCluster);
        }
    }

    // PCL Method
    /*
    typename pcl::search::KdTree<PointT>::Ptr tree( new pcl::search::KdTree<PointT> );
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (pcl::PointIndices getIndices : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster( new pcl::PointCloud<PointT> );

        for (int i : getIndices.indices) {
            cloudCluster->points.emplace_back(cloud->points[i]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.emplace_back(cloudCluster);
    }
    */

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


/************************************
 * MY ALGORITHMS
 ***********************************/

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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

			PointT pt = cloud->points[i];
			float dist = fabs(a*pt.x + b*pt.y + c*pt.z + d) / sqrt(a*a + b*b + c*c);
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
