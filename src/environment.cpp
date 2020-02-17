/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include <stdlib.h>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "cxxopts/include/cxxopts.hpp"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer, Vect3(0, 0, 2.6), cloud);
    //renderPointCloud(viewer, cloud, std::string("point cloud"));
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* processor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentPlaneRes = processor->SegmentPlane(cloud, 100, 0.2);
    renderPointCloud(viewer, segmentPlaneRes.first, std::string("obstacle"), Color(1, 0, 0));
    renderPointCloud(viewer, segmentPlaneRes.second, std::string("ground"), Color(0, 1, 0));
    delete lidar;
    delete processor;
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

cxxopts::ParseResult parse_args(int argc, char* argv[])
{
    std::vector<float> a = {.1, .2, .3};
    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
    ("m,max_iterations_ransac", "max number of iterations for ransac", cxxopts::value<int>()->default_value("100"))
    ("t,distance_tolerance_ransac", "distance tolerance for ransac", cxxopts::value<float>()->default_value("0.5"))
    ("c,cluster_tolerance_euclidean", "cluster tolerance for euclidean clustering", cxxopts::value<float>()->default_value("1"))
    ("n,min_cluster_size_euclidean", "min cluster size for euclidean clustering", cxxopts::value<int>()->default_value("50"))
    ("x,max_cluster_size_euclidean", "max cluster size for euclidean clustering", cxxopts::value<int>()->default_value("100"))
    ("r,filter_resolution", "filter resolution for point cloud", cxxopts::value<float>()->default_value("0.3"))
    ("i,roi_min", "roi_min_coordinates", cxxopts::value<std::vector<float>>()->default_value("-100,-5,-100"))
    ("j,roi_max", "roi_max_coordinates", cxxopts::value<std::vector<float>>()->default_value("100,7.5,100"));
    return options.parse(argc, argv);  
}

void process_pcd_stream(ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, std::string pcd_data_dir, pcl::visualization::PCLVisualizer::Ptr viewer, cxxopts::ParseResult args)
{
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(pcd_data_dir.c_str());
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI_filtered;
    //simpleHighway(viewer);
    int max_iterations_ransac = args["max_iterations_ransac"].as<int>();
    float distance_tolerance_ransac = args["distance_tolerance_ransac"].as<float>();
    std::vector<float> roi_min = args["roi_min"].as<std::vector<float>>();
    std::vector<float> roi_max = args["roi_max"].as<std::vector<float>>();
    float resolution = args["filter_resolution"].as<float>();
    float cluster_tolerance = args["cluster_tolerance_euclidean"].as<float>();
    int min_cluster_size = args["min_cluster_size_euclidean"].as<int>();
    int max_cluster_size = args["max_cluster_size_euclidean"].as<int>();

    while (!viewer->wasStopped ())
    {
            // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        inputCloudI_filtered = pointProcessorI->FilterCloud(inputCloudI, resolution, Eigen::Vector4f (roi_min[0], roi_min[1], roi_min[2],  1),  Eigen::Vector4f (roi_max[0], roi_max[1], roi_max[2], 1));
        std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentPlaneRes = pointProcessorI->SegmentPlane(inputCloudI_filtered, max_iterations_ransac, distance_tolerance_ransac);
        renderPointCloud(viewer, segmentPlaneRes.first, std::string("obstacle"), Color(1, 0, 0));
        renderPointCloud(viewer, segmentPlaneRes.second, std::string("ground"), Color(0, 1, 0));
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentPlaneRes.first, cluster_tolerance, min_cluster_size, max_cluster_size);
        int clusterId = 0;
        std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstcloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
            Box box= pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);

            ++clusterId;
        }
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();    
        viewer->spinOnce ();
    } 
}

int main (int argc, char* argv[])
{
    cxxopts::ParseResult args = parse_args(argc, argv);
    
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointCloudsCustom<pcl::PointXYZI>();
    process_pcd_stream(pointProcessorI, "../src/sensors/data/pcd/data_1", viewer, args);
}