/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* processorI,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    // ----------------------------------------------------
    // ----- Open 3D viewer and display city block    -----
    // ----------------------------------------------------
    
    // Filter point cloud data using voxels and region cropping
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = processorI->FilterCloud(
        inputCloud, 0.1f, Eigen::Vector4f(-15, -5, -3, 1), Eigen::Vector4f(80, 7, 5, 1));
    // renderPointCloud(viewer, filteredCloud, "filteredCloud");

    // Segment out Ground Plane
    auto segmentCloud = processorI->SegmentPlane(filteredCloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstacle_cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "plane_cloud", Color(0, 1, 0));

    // Cluster Obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processorI->Clustering(segmentCloud.first, 0.5, 10, 100000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    
    for (auto cluster : cloudClusters) {
        // Render Object Cloud
        // std::cout << "cluster size ";
        // processorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacles" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);

        // Render Bounding Box
        Box box = processorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor on ego vehicle
    Lidar* lidar1 = new Lidar(cars, 0.0);  // TODO: delete lidar1
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud1 = lidar1->scan();
    // renderRays(viewer, lidar1->position, pcloud1);
    renderPointCloud(viewer, pcloud1, "lidar1");

    // Segment out ground plane
    ProcessPointClouds<pcl::PointXYZ> processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = \
        processor.SegmentPlane(pcloud1, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstacle_cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "plane_cloud", Color(0, 1, 0));

    // Cluster obstacle point cloud into distinct objects
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = \
        processor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for (auto cluster : cloudClusters) {
        // Render Object Cloud
        std::cout << "cluster size ";
        processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacles" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);

        // Render Bounding Box
        Box box = processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* processorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = processorI->streamPcd("../src/src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {
        // Clear last frame
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load PCD data and run processing
        inputCloudI = processorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, processorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }

        viewer->spinOnce();
    } 
}