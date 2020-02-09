/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <random>
#include <iterator>
#include <algorithm>
#include <cmath>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

std::unordered_set<int> pickSet(int N, int k, std::mt19937& gen)
{
    std::unordered_set<int> elems;
    for (int r = N - k; r < N; ++r) {
        int v = std::uniform_int_distribution<>(1, r)(gen);

        // there are two cases.
        // v is not in candidates ==> add it
        // v is in candidates ==> well, r is definitely not, because
        // this is the first iteration in the loop that we could've
        // picked something that big.

        if (!elems.insert(v).second) {
            elems.insert(r);
        }   
    }
    return elems;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 100; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = scatter+i*i + 2*(((double) rand() / (RAND_MAX))-0.5);

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 20;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	int max_inliers = 0;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	std::vector<int> all_indices(cloud->points.size());
	std::iota(all_indices.begin(), all_indices.end(), 0);
	std::vector<int> sample(2);
	std::vector<pcl::PointXYZ> points(2);

	for(int j = 0; j < maxIterations; ++ j)
	{
		std::unordered_set<int> currResult;
		std::random_device rd;
		std::mt19937 gen(rd());
		std::unordered_set<int> sample = pickSet(cloud->points.size(), 2, gen);
		int l = 0;
		for(int k: sample){
			// std::cout << "k " << k << "j " <<  j << std::endl;
			points[l] = cloud->points[k];
			l++;
		}
		float A = points[0].y - points[1].y, B = points[1].x-points[0].x, C = points[0].x*points[1].y-points[0].y*points[1].x;
		float distance = std::abs(A*points[0].x + B*points[0].y + C)/std::sqrt(A*A + B*B);
		// std::cout << "distance!" << distance << std::endl;
		for(int k = 0; k < cloud->points.size(); ++k){
			pcl::PointXYZ pt = cloud->points[k];
			float distance = std::abs(A*pt.x + B*pt.y + C)/std::sqrt(A*A + B*B);
			// if (std::find(std::begin(sample), std::end(sample), k) != sample.end()) std::cout << "distance" << distance << std::endl;
			if(distance < distanceTol) currResult.insert(k);
		}
		if(currResult.size() > max_inliers)
		{
			max_inliers = currResult.size();
			inliersResult = currResult;
		}
	}
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	int max_inliers = 0;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	std::vector<int> all_indices(cloud->points.size());
	std::iota(all_indices.begin(), all_indices.end(), 0);
	std::vector<int> sample(3);
	std::vector<pcl::PointXYZ> pts(3);

	for(int j = 0; j < maxIterations; ++ j)
	{
		std::unordered_set<int> currResult;
		std::random_device rd;
		std::mt19937 gen(rd());
		std::unordered_set<int> sample = pickSet(cloud->points.size(), 3, gen);
		int l = 0;
		for(int k: sample){
			pts[l] = cloud->points[k];
			l++;
		}
		float A = (pts[1].y - pts[0].y)*(pts[2].z-pts[0].z) - (pts[2].y-pts[0].y)*(pts[1].z-pts[0].z);
		float B = (pts[2].x - pts[0].x)*(pts[1].z-pts[0].z) - (pts[1].x-pts[0].x)*(pts[2].z-pts[0].z);
		float C = (pts[1].x - pts[0].x)*(pts[2].y-pts[0].y) - (pts[2].x-pts[0].x)*(pts[1].y-pts[0].y);
		//continue to next iteration if points are collinear 
		if(A==0 && B==0 && C==0) continue;
		float D = -(A*pts[0].x + B*pts[0].y + C*pts[0].z);
		float distance = std::abs(A*pts[1].x + B*pts[1].y + C*pts[1].z + D)/std::sqrt(A*A + B*B + C*C);
		// std::cout << "distance!" << distance << std::endl;
		for(int k = 0; k < cloud->points.size(); ++k){
			pcl::PointXYZ pt = cloud->points[k];
			float distance = std::abs(A*pt.x + B*pt.y + C*pt.z + D)/std::sqrt(A*A + B*B + C*C);
			// if (std::find(std::begin(sample), std::end(sample), k) != sample.end()) std::cout << "distance" << distance << std::endl;
			if(distance < distanceTol) currResult.insert(k);
		}
		if(currResult.size() > max_inliers)
		{
			max_inliers = currResult.size();
			inliersResult = currResult;
		}
	}
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 1000, 0.01);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	// std::cout << "here!" << std::endl;
	// std::cout << inliers.size() << std::endl;
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
