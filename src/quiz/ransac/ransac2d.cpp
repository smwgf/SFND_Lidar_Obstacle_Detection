/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
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
	srand(time(NULL));
	
	// TODO: Fill in this function

	
	std::unordered_set<int>* mostInlier (new std::unordered_set<int>());
	

	// For max iterations 
	for(int i =0 ; i<maxIterations;i++)
	{
		std::unordered_set<int>* currentInlier (new std::unordered_set<int>());
		// Randomly sample subset and fit line
		int index1 = rand()%cloud->size();
		int index2 = rand()%cloud->size();
		if(index1==index2){
			index2=index1+1;
			if(index2>=cloud->size()){
				index2=0;
			}
		}
		float x1 =cloud->points[index1].x;
		float x2 =cloud->points[index2].x;
		float y1 =cloud->points[index1].y;
		float y2 =cloud->points[index2].y;
		float A = y1-y2;
		float B = x2-x1;
		float C = x1*y2 - x2*y1;
		float sqrA = A*A;
		float sqrB = B*B;

		// Measure distance between every point and fitted line
		for(int j=0 ; j <cloud->size();j++)
		{			
			float x =cloud->points[j].x;
			float y =cloud->points[j].y;
			float d = fabs(A*x+B*y+C)/sqrt(sqrA+sqrB);
			// If distance is smaller than threshold count it as inlier
			if(d<distanceTol)
			{
				currentInlier->insert(j);
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

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	
	std::unordered_set<int>* mostInlier (new std::unordered_set<int>());
	

	// For max iterations 
	for(int i =0 ; i<maxIterations;i++)
	{
		std::unordered_set<int>* currentInlier (new std::unordered_set<int>());
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
				currentInlier->insert(j);
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 100, 0.4);
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.4);	

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
