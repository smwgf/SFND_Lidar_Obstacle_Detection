// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
//#define USE_PCL
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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_crop_filtered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloud_filtered);

    pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloud_filtered);
    boxFilter.filter(*cloud_crop_filtered);


    pcl::CropBox<PointT> egoFilter;
    
    std::vector<int> indices;
    egoFilter.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    egoFilter.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    egoFilter.setInputCloud(cloud_crop_filtered);
    egoFilter.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());;

    for(int d : indices)
    {
        inliers->indices.push_back(d);
    }

    pcl::ExtractIndices<PointT> extract;    
    extract.setInputCloud(cloud_crop_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_crop_filtered);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_crop_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>());
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);

    extract.setNegative(true);
    extract.filter(*cloud_f);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}

#ifdef USE_PCL
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size()==0)
    {
        std::cerr << "Could not estimate a planner model for the given dataset." <<std::endl;        
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
#else
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
 
    inliers->indices = Ransac3DPlane(cloud,maxIterations,distanceThreshold);

    if(inliers->indices.size()==0)
    {
        std::cerr << "Could not estimate a planner model for the given dataset." <<std::endl;        
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
#endif

#ifdef USE_PCL
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

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*

        clusters.push_back(cloud_cluster);
    }  
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
#else
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

	KdTree* kdtree = new KdTree;
  
    kdtree->InsertPointCloud<PointT>(cloud);

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud,kdtree,clusterTolerance);

    for (std::vector<int> indice : cluster_indices)
    {        
        if((indice.size()>minSize) && (indice.size()<maxSize))
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
            for (int i : indice)
                cloud_cluster->points.push_back (cloud->points[i]); //*

            clusters.push_back(cloud_cluster);
        }
    }  
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
#endif

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

template<typename PointT>
std::vector<int> ProcessPointClouds<PointT>::Ransac3DPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::vector<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	
	std::vector<int>* mostInlier (new std::vector<int>());
	

	// For max iterations 
	for(int i =0 ; i<maxIterations;i++)
	{
		std::vector<int>* currentInlier (new std::vector<int>());
		// Randomly sample subset and fit line
		int index1 = rand()%cloud->size();
		int index2, index3;
		do{
			index2 = rand()%cloud->size();
		}
		while(index1==index2);
		do{
			index3 = rand()%cloud->size();
		}
		while(index1==index3||index2==index3);
		
		float x1 =cloud->points[index1].x;
		float x2 =cloud->points[index2].x;
		float x3 =cloud->points[index3].x;
		float y1 =cloud->points[index1].y;
		float y2 =cloud->points[index2].y;
		float y3 =cloud->points[index3].y;
		float z1 =cloud->points[index1].z;
		float z2 =cloud->points[index2].z;
		float z3 =cloud->points[index3].z;
		float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float D = - A*x1 - B*y1 - C*z1;
		float sqrA = A*A;
		float sqrB = B*B;
		float sqrC = C*C;

		// Measure distance between every point and fitted line
		for(int j=0 ; j <cloud->size();j++)
		{			
			float x =cloud->points[j].x;
			float y =cloud->points[j].y;
			float z =cloud->points[j].z;
			float d = fabs(A*x+B*y+C*z +D)/sqrt(sqrA+sqrB+sqrC);
			// If distance is smaller than threshold count it as inlier
			if(d<distanceTol)
			{
				currentInlier->push_back(j);
			}			
		}
		if(mostInlier->size()<currentInlier->size())
		{
			delete mostInlier;
			mostInlier=currentInlier;
		}
		else
		{
			delete currentInlier;
		}
		

	}
	// Return indicies of inliers from fitted line with most inliers
	return *mostInlier;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(std::vector<bool>& processed,typename pcl::PointCloud<PointT>::Ptr cloud,int id,std::vector<int>& cluster,KdTree* tree, float distanceTol)
{
	processed[id]=true;
	cluster.push_back(id);
    auto d = cloud->points[id];
    std::vector<float> data = {d.x,d.y,d.z};
	std::vector<int> ids = tree->search(data,distanceTol);
	for(int i : ids)
	{
		if(!processed[i])
		{
			proximity(processed,cloud,i,cluster,tree,distanceTol);
		}
	}
}
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed = std::vector<bool>(points->size(),false);
   	for(int i=0 ; i < points->size();i++)
	{		
		//if point has not been processed
		if(!processed[i])
		{
			std::vector<int>& cluster (*(new std::vector<int>()));
			proximity(processed,points,i,cluster,tree,distanceTol);
			clusters.push_back(cluster);
		}
	}
	return clusters;
}