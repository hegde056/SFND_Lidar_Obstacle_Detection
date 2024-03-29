// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


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
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());

    //create the filtering object 
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    //return cloudFiltered;

    //cloud filtered is not working?
    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for(int index:inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers , *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr<<"Could not estimate a planar model for the given dataset."<<std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations--)
	{
		//Randomly sample subset and fit line 
		std::unordered_set <int> inliers;
		while(inliers.size() < 3) //3D plane - dimension ->3  
			inliers.insert(rand()%(cloud->points.size()));
		
		//fitting the line
		float x1,y1,z1,x2,y2,z2,x3,y3,z3;

		auto itr = inliers.begin();
		x1  = cloud->points[*itr].x;
		y1  = cloud->points[*itr].y;
		z1  = cloud->points[*itr].z;
		itr++;
		x2  = cloud->points[*itr].x;
		y2  = cloud->points[*itr].y;
		z2  = cloud->points[*itr].z;
		itr++;
		x3  = cloud->points[*itr].x;
		y3  = cloud->points[*itr].y;
		z3  = cloud->points[*itr].z;

		float  i ,j,k;
		i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);


		float a = i;
		float b = j;
		float c = k;
		float d = -1 * ((i*x1) + (j*y1) + (k*z1));

		//measuring distance btw every point and fitted line
		for(int index = 0 ; index < cloud->points.size(); index++)
		{
			if (inliers.count(index)>0)
				continue;
			PointT point  = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float dist = fabs((a*x4 + b*y4 + c*z4 + d )/sqrt(a*a + b*b + c*c));
			// If distance is smaller than threshold count it as inlier
			if (dist <= distanceThreshold)
			{
				inliers.insert(index);
			}

			// Return indicies of inliers from fitted line with most inliers
			if(inliers.size() > inliersResult.size())
			{
				inliersResult = inliers;
			}

		}

	}
 
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	} 

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl; 

    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (clusterTolerance); // 2cm<-->0.02
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (maxSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (clusterIndices);
  
  for(pcl::PointIndices getIndices :  clusterIndices )
  {
    typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
    for (int index : getIndices.indices)
    {
      cloudCluster->points.push_back(cloud->points[index]);
    }
    cloudCluster->width = cloudCluster->points.size ();
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
void ProcessPointClouds<PointT>::clusterHelper(int index,const std::vector<std::vector<float>> points,std::vector<int>& cluster,std::vector<bool>& processed,KdTree* tree,float distanceTol)
{
	//mark point as processed 
	processed[index] = true;
	//add point to cluster 
	cluster.push_back(index);
	//get nearby points 
	std::vector<int> nearbyPts  = tree->search(points[index], distanceTol);
	//iterate through each nearby point 
	for(int id:nearbyPts)
	{
		if(!processed[id])
		//recursively call the helper for unprocessed points 
			clusterHelper(id, points, cluster, processed, tree, distanceTol );
	}



}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int  minSize,int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	//create a list of clusters
	std::vector<std::vector<int>> clusters;

	//create a vector for processed points . type: bool
	std::vector<bool> processed(points.size(), false);

	int i=0;
	//iterate through each point
	while (i<points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}

		//acc. to the algo -->
		//if the point has not been processed, 
		//create a cluster 
		std::vector<int> cluster;

		//Proximity (point, cluster)-> implement this using clusterhelper
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
        if(cluster.size()>=minSize && cluster.size()<=maxSize)
        {
		    //cluster add clusters 
		    clusters.push_back(cluster);
        }

		i++;
	}
 
	return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::eucledianClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    std::vector<std::vector<float>> points;
    std::vector<float> point(3);
    for(int index = 0; index < cloud->points.size(); index++)
        {
            point[0] = cloud->points[index].x;
            point[1] = cloud->points[index].y;
            point[2] = cloud->points[index].z;
            points.push_back(point);
            
        } 

    //points visualization
    // for(int index = 0; index < points.size(); index++)
    //     {
    //         std::cout<<" {";
    //         for(int i = 0; i < points[index].size(); i++) 
    //         {
    //             std::cout<<points[index].at(i)<<','; 
    //         }
    //         std::cout<<" }";
    //     }

	KdTree* tree = new KdTree;
    tree->kDim = 3;
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i);

    std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, clusterTolerance , minSize, maxSize);
  	

  	for(std::vector<int> cluster : clusterIndices)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(std::vector<int>::const_iterator itr  = cluster.begin(); itr != cluster.end(); ++itr)
  			clusterCloud->points.push_back(cloud->points[*itr]);
        clusterCloud->width = clusterCloud->points.size ();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
    
        clusters.push_back(clusterCloud);

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