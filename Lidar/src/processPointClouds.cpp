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

    /* This function do downsampling by using VoxelBox (VoxelBox choose the resolution of the pointCloud)
        it then define the min and max region that we want in the PCD (region of interest)
        Then, finally it remove the roof of the car (optionally)
    */



    //The vectors here define the min and max of the boudary space
    // we don't want to get data that is too far. We need to choose the region of interest.

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> );
    //filter the cloud as in (reducing the size using cubic resolution)
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (filterRes, filterRes, filterRes);//choose the dimension of the cubic resolution.
    vox.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT> );//this is for the region of interest

    pcl::CropBox<PointT> region(true); //true means that when we use extract, we will remove the given indices from the cloud
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region); //This will cut cloud_filtered to the region specified by minPoint and maxPoint

    //cut the roof of the car (ensure that the car is not part of the cloud)

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true); //true means that when we use extract, we will remove the given indices from the cloud
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1)); //min in x,y,z
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1)); // maz in x,y,z
    roof.setInputCloud(cloud_region);
    //There are two types of filter function, one takes point cloud input
    // and one that save the points indices instead 
    roof.filter(indices);

    //now we need to convert the vector to PointIndices type so that we can extract them from the point cloud
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); 

    for (int index:indices){
        inliers->indices.push_back(index);
    }

    //create the extract object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    //the inliers contains the indices of the points generated from RANSAC (for plane in this case)
    // we neeed to save in a point cloud formats
    //inliers are vectors of indeces that is part of the plane
    for (int index: inliers->indices)
    {
        //save each points in the inliers to placeCloud
        planeCloud->points.push_back(cloud->points[index]); 
    }
    
    pcl::ExtractIndices<PointT> extract; // create an extract object that will give the PCD of the planes

    //the extract object is used to generate the obsticle detection (filter the plane and you will be left with the obstilce)
    //Extract the inliers (the only plane in this case is the road)
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);//in the indeces of pointed that we want to filter (remove)
    extract.setNegative (true);
    extract.filter (*obstCloud);// save the filtered PCD to this variable
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult( obstCloud,planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg; //object for implementing segmentation of PCD


    //the segmentation algoritms is used to generate the inlliers and the coefficients
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); // the variable that is used to seperate the planes
   

    // set up the segmentation parameters
    seg.setOptimizeCoefficients (true); //this is optional
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);//we want to segment planes/ but this could be cylinders, cirles, lines etc
    seg.setMethodType (pcl::SAC_RANSAC);//the algorithms is RANSAC 
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    //finally we just need to provide the input PCD and save the segments result in inliers and coefficients
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    // if (inliers->indices.size() == 0)
    // {
    //     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    //     return (-1);
    // }


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; // vectors of clusters, each cluster is in a sperate Point cloud

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices; //this a vector of int vectors : will save the indeces of the clusters 
    //note cluster_indices[0] contains all the indeces of the first cluster

    //create a euclidean cluster extraction object and specify the params
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree); // the search method is tree (instance of KdTree)
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);// here is where the cluster_indeces is filled

    //now, we need to save the clustered points into a point cloud variables \

    for (pcl::PointIndices getIndices :cluster_indices){//save the indeces for each cluster in a variavle called getIndeces

        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new typename pcl::PointCloud<PointT>); //will be pushed back to clusters variable (define at the start)

        for (int index:getIndices.indices ){//extract each index to create a Point cloud variable for each cluster
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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	
    
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
        float D = -1*(A*x1+B*y1+C*z1);

        // Measure distance between every point and fitted plane
        for (int index=0; index < cloud->points.size();index++){

            if (inliers.count(index)){ // test if this is one of the smaple points, if yes , continue
                continue; 
            }

            float x4= cloud->points[index].x;
            float y4= cloud->points[index].y;
            float z4= cloud->points[index].z;

            float distance = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);
            if (distance <= distanceThreshold ){

                inliers.insert(index);
            }

        }
        if (inliersResult.size()<inliers.size()){
            inliersResult=inliers; //in each iteration, the size of of inliers will be different. 
            //here, we every time the inliers has more points we set it as the result.
            //We want the plane with the largest inliers.
        }


    }
    //std::cout<<"max inliers are: " << inliersResult.size()<<"\n";
    typename pcl::PointCloud<PointT>::Ptr  plane(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obstacle(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			plane->points.push_back(point);
		else
			obstacle->points.push_back(point);
	}


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, plane);
	
	return segResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::cluster_Helper(int index,const std::vector<std::vector<float>>& points,std::vector<bool> &processed, std::vector<int> &cluster ,KdTree* tree,float distanceTol){

	processed [index]=true; //mark this point as processed

	cluster.push_back(index);// add the index to the cluster

	std::vector<int> nearPoints = tree->search(points[index],distanceTol );

	for (int id :nearPoints){

		if (!processed[id])
			cluster_Helper (id, points,processed,cluster,tree,distanceTol);
	}

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclidean_Cluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters; 
	std::vector<bool> processed (points.size(),false);

	int i=0; 

	while (i < points.size()){

		if (processed[i]){ // if the points is already processed, pass
			i++;
			continue;

		}
		std::vector<int> cluster; 
		cluster_Helper (i, points,processed,cluster,tree,distanceTol);
		clusters.push_back(cluster);
		i++;
	}
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_from_scratch(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;//return variable
    KdTree* tree = new KdTree;

    //convert all the point cloud to vecotor of float vector so that we can use KdTree previously defined struct
    //this is neccesary to avoid datatype collision
    std::vector<std::vector<float>> cloud_float;
    for (int i=0; i <cloud->points.size(); i++){
        std::vector<float> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        point.push_back(cloud->points[i].z);
        cloud_float.push_back(point);
        tree->insert(cloud_float[i],i); 
    }

    //create the tree
    // for (int i=0; i<cloud->points.size(); i++) {
    //     //convert the cloud point to 
    //     //The conversion was required because the insert fucntion accept a vector of float only
        
    	
    // }

    //do the euclidean clustering
    std::vector<std::vector<int>> cluster_indices = ProcessPointClouds<PointT>::euclidean_Cluster (cloud_float,tree, clusterTolerance);

   
    // //convert the indices to a pointcloud and push it back to the clusters variable
    for (std::vector<int> cluster : cluster_indices){//save the indeces for each cluster in a variavle called getIndeces
        
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new typename pcl::PointCloud<PointT>); //will be pushed back to clusters variable (define at the start)

        for (int index:cluster ){//extract each index to create a Point cloud variable for each cluster
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        //check if the cluster size is within the min/max
        if(cloudCluster->points.size()> minSize && cloudCluster->points.size()<maxSize){
            clusters.push_back(cloudCluster);
         }
    }

    return clusters;
}