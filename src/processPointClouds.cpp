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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_no_ego_roof(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxelizer;
    voxelizer.setInputCloud(cloud);
    voxelizer.setLeafSize(filterRes, filterRes, filterRes);
    voxelizer.filter(*cloud_filtered);

    pcl::CropBox<PointT> cropper(true);
    cropper.setMin(minPoint);
    cropper.setMax(maxPoint);
    cropper.setInputCloud(cloud_filtered);
    cropper.filter(*cloud_cropped);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof_cropper(true);
    roof_cropper.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof_cropper.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof_cropper.setInputCloud(cloud_cropped);
    roof_cropper.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point: indices) 
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_ego_roof);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_no_ego_roof;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    typename pcl::PointCloud<PointT>::Ptr cloud_n(new pcl::PointCloud<PointT>);
    extract.filter(*cloud_n);
    typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>);
    extract.setNegative(false);
    // for(int index: inliers->indices)
    //     cloud_p->points.push_back(cloud->points[index]);
    extract.filter(*cloud_p);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_n, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold); 
    seg.setMaxIterations(maxIterations);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        exit(1);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for(int index: getIndices.indices) cloudCluster->points.push_back(cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }
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

//******************* Use custom implementation of RANSAC**************

template<typename PointT>
ProcessPointCloudsCustom<PointT>::ProcessPointCloudsCustom() {}


//de-constructor:
template<typename PointT>
ProcessPointCloudsCustom<PointT>::~ProcessPointCloudsCustom() {}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointCloudsCustom<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inlier_indices = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
    if (inlier_indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        exit(1);
    }
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    inliers->indices.assign(inlier_indices.begin(), inlier_indices.end());
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = this->SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointCloudsCustom<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> points;
    KdTree* tree = new KdTree;

    for (int i =0; i < cloud->points.size(); ++i){
        PointT point = cloud->points[i];
        std::vector<float> v_point{point.x, point.y, point.z};
        points.push_back(v_point);
        tree->insert(v_point, i);
    }
    std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance);
    for(std::vector<int> indices: cluster_indices)
    {
        std::cout << "size " << indices.size() << std::endl;
        if (indices.size() < minSize) continue;
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index: indices) cloudCluster->points.push_back(cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }
    std::cout << "done" << std::endl;
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    delete tree;
    return clusters;
}

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

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
	std::vector<PointT> points(2);

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
			PointT pt = cloud->points[k];
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

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
	// std::vector<int> all_indices(cloud->points.size());
	// std::iota(all_indices.begin(), all_indices.end(), 0);
	std::vector<int> sample(3);
	std::vector<PointT> pts(3);

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
		if(A==0 && B==0 && C==0) continue;
		float D = -(A*pts[0].x + B*pts[0].y + C*pts[0].z);
		float distance = std::abs(A*pts[1].x + B*pts[1].y + C*pts[1].z + D)/std::sqrt(A*A + B*B + C*C);
		for(int k = 0; k < cloud->points.size(); ++k){
			PointT pt = cloud->points[k];
			float distance = std::abs(A*pt.x + B*pt.y + C*pt.z + D)/std::sqrt(A*A + B*B + C*C);
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