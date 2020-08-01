/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
//#include "sensors/lidar.h"\

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; //making this false will remove the scene (cars, street)
    bool render_obst = true;
    bool render_plane =true;
    bool render_cluster =true;
    bool render_box =true; 

    std::vector<Car> cars = initHighway(renderScene, viewer);
    // TODO:: Create lidar sensor 
    Lidar *lidar= new Lidar(cars,0); // cars comming from the highway fuction, and the slope for the ground plane is 0
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer,lidar->position, cloud); // visualize rays
    //renderPointCloud(viewer, cloud,"Input cloud"); // visualize points cloud
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processCloud ;
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud= processCloud.SegmentPlane(cloud,100,0.2);
    
    if(render_obst)
        renderPointCloud(viewer,segmentCloud.first,"Obsticle plane",Color(1,0,0));

    if (render_plane)    
        renderPointCloud(viewer, segmentCloud.second,"Plane cloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processCloud.Clustering(segmentCloud.first, 1.0, 3, 30); // cluster the obsticle segmentation
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processCloud.numPoints(cluster);

        if (render_cluster)
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        
        Box box = processCloud.BoundingBox(cluster);
        if (render_box)
            renderBox(viewer,box,clusterId);
        
        ++clusterId;
    }
    
}



void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    bool render_obst = true;
    bool render_plane =true;
    bool render_cluster =true;
    bool render_box =true;

    //parameters 
    int maxIterations= 100;
    float distanceThreshold =0.2;
    float clusterTolerance=0.45;
    int minSize=5;
    int maxSize =300;

    //renderPointCloud(viewer,inputCloud,"inputCloud");

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = pointProcessorI->FilterCloud(inputCloud, 0.3f , Eigen::Vector4f (-20, -6, -3, 1), Eigen::Vector4f ( 30, 7, 0, 1));
    //renderPointCloud(viewer,filtered_cloud,"filterCloud");

    //segementation
    //std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud= pointProcessorI->SegmentPlane(filtered_cloud,maxIterations,distanceThreshold);
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud= pointProcessorI->ransac3D(filtered_cloud,maxIterations,distanceThreshold);
    if(render_obst)
        renderPointCloud(viewer,segmentCloud.first,"Obsticle plane",Color(1,0,0));//red
        //std::cout<<"the size of obstacle : "<< segmentCloud.first->points.size()<<endl;
    if (render_plane)    
        renderPointCloud(viewer, segmentCloud.second,"Plane cloud", Color(0,1,0)); //green

    //std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 5, 500);//cluster the car
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering_from_scratch(segmentCloud.first, clusterTolerance, minSize, maxSize);
    //show the cluster in different colors (not uniques)
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1),Color(0,1,1)};
    std::cout << "number of clusters "<<cloudClusters.size() <<"\n";
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);

        if (render_cluster)
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[0]);
        
        Box box = pointProcessorI->BoundingBox(cluster);
        if (render_box)
            renderBox(viewer,box,clusterId,colors[0]);
        
        ++clusterId;
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
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    

    //we are creating a vector of all the files in the folder data1. the stream will have all files reordered by time
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();//start an itrator pointing at the first file 
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI ( new pcl::PointCloud<pcl::PointXYZI> ) ;
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);


        streamIterator++;
        if(streamIterator == stream.end()){
            
            streamIterator = stream.begin();
        }
            
        viewer->spinOnce ();
    } 
}