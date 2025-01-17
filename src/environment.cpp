/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "SFND.h"
#include "SFND.cpp"

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer);

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
    Lidar *lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();


    renderRays(viewer,lidar->position,inputCloud);
    renderPointCloud(viewer,inputCloud,"inputcloud");

    pcl::visualization::PCLVisualizer::Ptr viewer1 (new pcl::visualization::PCLVisualizer ("plane and obstacle cloud"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer1);

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    renderPointCloud(viewer1,segmentCloud.second,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer1,segmentCloud.first,"planeCloud",Color(1,1,1));

   pcl::visualization::PCLVisualizer::Ptr viewer2 (new pcl::visualization::PCLVisualizer ("clusters"));

   initCamera(setAngle, viewer2);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.second, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointProcessor.numPoints(cluster);
          renderPointCloud(viewer2,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          ++clusterId;
          Box box = pointProcessor.BoundingBox(cluster);
          renderBox(viewer2,box,clusterId);
    }

while (!viewer->wasStopped ())viewer->spinOnce ();

}

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> * pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor->loadPcd("../SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessor->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f (30, 8, 1, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(cloudFiltered, 50, 0.3);

    renderPointCloud(viewer, segmentCloud.first, "palneCloud", Color(0, 1, 0));
    //renderPointCloud(viewer, segmentCloud.second, "obstCloud", Color(1, 0, 0));

    // cluster
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.second, 0.8, 10, 500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0),Color(1, 1, 0), Color(0, 0, 1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "Cluster Size: ";
        pointProcessor->numPoints(cluster);
       renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessor->BoundingBox(cluster);

        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

  //renderPointCloud(viewer,cloudFiltered,"cloudFiltered");

//  Box egocar;
//      egocar.x_max = 2.6;
//      egocar.y_max = 1.7;
//      egocar.z_max = -0.4;
//      egocar.x_min = -1.5;
//      egocar.y_min = -1.7;
//      egocar.z_min = -1;
//      renderBox(viewer, egocar, 1, Color(0.5,0.,0.5));

}

void SFND_CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, SFND<pcl::PointXYZI> * pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr const inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor->loadPcd("../SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessor->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f (30, 8, 1, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(cloudFiltered, 20, 0.3);

    renderPointCloud(viewer, segmentCloud.first, "palneCloud", Color(0, 1, 0));
    renderPointCloud(viewer, segmentCloud.second, "obstCloud", Color(1, 0, 0));

    // cluster
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.second, 0.8, 10, 500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0),Color(1, 1, 0), Color(0, 0, 1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "Cluster Size: ";
        pointProcessor->numPoints(cluster);
       renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessor->BoundingBox(cluster);

        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

  //renderPointCloud(viewer,cloudFiltered,"cloudFiltered");

//  Box egocar;
//      egocar.x_max = 2.6;
//      egocar.y_max = 1.7;
//      egocar.z_max = -0.4;
//      egocar.x_min = -1.5;
//      egocar.y_min = -1.7;
//      egocar.z_min = -1;
//      renderBox(viewer, egocar, 1, Color(0.5,0.,0.5));

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

    if(argc<3){
     std::cout<<"Please specify the function to execute \n"<<
                "./environment highway -\n"<<
                "./environment cityblock_pcl filename \n" <<
                "./environment cityblock filename\n";

     return 0;

    }

    if(!strcmp(argv[1],"highway")){
    simpleHighway(viewer);
    return 0;
    }


    if(!strcmp(argv[1],"cityblock_pcl")){
        //cityBlock(viewer);
        ProcessPointClouds<pcl::PointXYZI> * pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>;
        std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(argv[2]);
        auto streamIterator = stream.begin();

        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

        while (!viewer->wasStopped ())
        {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            CityBlock(viewer, pointProcessorI, inputCloudI);
            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin();
            //usleep(100000);
            viewer->spinOnce ();

        }
        return 0;
    }

    if(!strcmp(argv[1],"cityblock")){
        //cityBlock(viewer);
        SFND<pcl::PointXYZI> * pointProcessorI = new SFND<pcl::PointXYZI>;
        std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(argv[2]);
        auto streamIterator = stream.begin();

        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

        while (!viewer->wasStopped ())
        {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            SFND_CityBlock(viewer, pointProcessorI, inputCloudI);
            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin();
            //usleep(100000);
            viewer->spinOnce ();

        }
        return 0;
    }

    std::cout<<"Please specify the function to execute \n"<<
               "./environment highway \n"<<
               "./environment cityblock_pcl \n" <<
               "./environment cityblock \n";

    return 0;

}
