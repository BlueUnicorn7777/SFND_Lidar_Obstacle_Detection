// PCL lib Functions for processing point clouds 

#include "SFND.h"


//constructor:
template<typename PointT>
SFND<PointT>::SFND() {}


//de-constructor:
template<typename PointT>
SFND<PointT>::~SFND() {}


template<typename PointT>
void SFND<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}



template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr SFND<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr const cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(filterRes, filterRes, filterRes);
        vg.filter(*cloudFiltered);

        typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
        pcl::CropBox<PointT> region(true);
        region.setMin(minPoint);
        region.setMax(maxPoint);
        region.setInputCloud(cloudFiltered);
        region.filter(*cloudRegion);

        std::vector<int> indices;
        pcl::CropBox<PointT> roof;
        roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
        roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
        roof.setInputCloud(cloudRegion);
        roof.filter(indices);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        for(int point : indices)
            inliers->indices.push_back(point);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloudRegion);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SFND<PointT>::SeparateClouds( typename std::unordered_set<int> const  inliers, typename pcl::PointCloud<PointT>::Ptr const cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
   typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
   typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index)) planeCloud->points.push_back(point);
        else obstCloud->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud,obstCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SFND<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr const cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    std::unordered_set<int> inliers;
    double x1, y1, z1, x2, y2, z2, x3, y3, z3 ,a,b,c,d;

    auto iter = inliers.begin();
    float distanceThreshold_redef;
    while(maxIterations--)
    {
        inliers.clear();
        while (inliers.size() < 3)
            inliers.insert(rand()%cloud->points.size());

        iter = inliers.begin();

        x1 = cloud->points[*iter].x;        y1 = cloud->points[*iter].y;        z1 = cloud->points[*iter].z;
        iter++;
        x2 = cloud->points[*iter].x;        y2 = cloud->points[*iter].y;        z2 = cloud->points[*iter].z;
        iter++;
        x3 = cloud->points[*iter].x;        y3 = cloud->points[*iter].y;        z3 = cloud->points[*iter].z;

         a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
         b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
         c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
         d = -(a*x1 + b*y1 + c*z1);

         distanceThreshold_redef= distanceThreshold * sqrt(a*a + b*b + c*c);

        for(unsigned long int i = 0; i < cloud->points.size(); ++i)
        {
            if(inliers.count(i) > 0)
                continue;
            if(fabs(a*cloud->points[i].x + b*cloud->points[i].y + c*cloud->points[i].z + d) <= distanceThreshold_redef)
                inliers.insert(i);
        }

        if(inliersResult.size() < inliers.size())
            inliersResult = inliers;
    }



    if(inliersResult.size()==0){
        std::cout<<"Could not find a planer model for a given dataset."<<std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResults = SeparateClouds(inliersResult,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResults;
}

template<typename PointT>
void clusterHelper(int index,  typename pcl::PointCloud<PointT>::Ptr  const cloud,
                   typename pcl::PointCloud<PointT>::Ptr &cluster, std::vector<bool> &processed, KdTree<PointT> *tree, float distanceTol,int maxSize)
{
    processed[index] = true;
    cluster->points.push_back(cloud->points[index]);
   // if(cluster->points.size()>maxSize)return;

    std::vector<int> nearest = tree->search(cloud->points[index], distanceTol);
    for (int id : nearest)
    {
        if (processed[id] == false)
            clusterHelper(id, cloud, cluster, processed, tree, distanceTol, maxSize);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> SFND<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr  const cloud , float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    KdTree<PointT> *tree = new KdTree<PointT>;

    for(int index = 0; index < cloud->points.size(); index++)
    tree->insert(cloud->points[index],index);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(), false);
    for(int index = 0; index < cloud->points.size(); index++){
         if(processed[index]) continue;
         typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>());
         clusterHelper(index, cloud, cluster, processed, tree, clusterTolerance,maxSize);
        if(cluster->points.size()>minSize)
           clusters.push_back(cluster);
    }
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    delete tree;

    return clusters;
}


template<typename PointT>
Box SFND<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void SFND<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr SFND<PointT>::loadPcd(std::string file)
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
std::vector<boost::filesystem::path> SFND<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
