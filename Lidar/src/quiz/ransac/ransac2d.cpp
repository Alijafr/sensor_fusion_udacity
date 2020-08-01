/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
	
	//Ransac for a line
	/*
	// TODO: Fill in this function

	// For max iterations 
	for (int i=0; i<maxIterations; i++){
		std::unordered_set<int> inliers;// the unorder set is needed to reject duplicate smaple of points
	
	// Randomly sample subset and fit line
		while (inliers.size()<2)
		{
			inliers.insert(rand()%(cloud->points.size()));//return random index from 0 to the size of the points cloud points
		}
		
		//extract the points at the sample indeces 

		float x1,x2,y1,y2;

		auto itr=inliers.begin(); // create a pointer that points to the begining the unorder_set
		//you need to dereference the pointer to obtain the value.
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		itr++;
		x2=cloud->points[*itr].x;
		y2=cloud->points[*itr].y;

		//calculate a,b,c to find the distance between points and line.
		float a,b,c;
		a=(y1-y2);
		b=(x2-x1);
		c=(x1*y2)-(x2*y1);

		//we need to check what points is part of the line and which is not
		for (int i=0; i < cloud->points.size(); i++){
			//before we start, we need to make sure that the point is not part of the smaple 2 points 

			if(inliers.count(i)){ // if the element already in inliers set, the result will be biger than 0 (true)
				continue;	
			}


			float x3=cloud->points[i].x;
			float y3=cloud->points[i].y;
			float d = fabs(a*x3+b*y3)/sqrt(a*a+b*b);

			if (d<distanceTol){ //check if the points is within the tolerance` 
				inliers.insert(i); //add the points to the inliers 	
			}
			if (inliersResult.size()<inliers.size()){
				inliersResult=inliers;
			}


		}
		}
		 */
		
		//ransac for plane

		for (int i=0; i < maxIterations;i++){
			std::unordered_set<int> inliers; // a variable that hold the points for the planes (will be overwriten after each iteration)

			//sample three points (min model of a plane)
			while(inliers.size() <3){
				inliers.insert(rand()%(cloud->points.size()));
			}

			auto itr = inliers.begin();

			float x1,y1,z1,x2,y2,z2,x3,y3,z3;

			x1=cloud->points[*itr].x;
			y1=cloud->points[*itr].y;
			z1=cloud->points[*itr].z;
			itr++;
			x2=cloud->points[*itr].x;
			y2=cloud->points[*itr].y;
			z2=cloud->points[*itr].z;
			itr++;
			x3=cloud->points[*itr].x;
			y3=cloud->points[*itr].y;
			z3=cloud->points[*itr].z;

			float A=(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
			float B =(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
			float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
			float D = -(A*x1+B*y1+C*z1);

			for (int index=0; index <cloud->points.size();index++){

				if (inliers.count(index)){ // test if this is one of the smaple points, if yes , continue
					continue; 
				}

				float x4= cloud->points[index].x;
				float y4= cloud->points[index].y;
				float z4= cloud->points[index].z;

				float distance = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);

				if (distance < distanceTol ){

					inliers.insert(index);
				}

			}
			if (inliersResult.size()<inliers.size()){
				inliersResult=inliers;
			}



		}
		

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 1000, 0.4);

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
