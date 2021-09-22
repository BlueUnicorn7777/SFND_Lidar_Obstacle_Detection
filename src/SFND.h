// PCL lib Functions for processing point clouds 

#ifndef SFND_H_
#define SFND_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

template<typename PointT>
class SFND {
public:

    //constructor
    SFND();
    //deconstructor
    ~SFND();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr const cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(typename std::unordered_set<int> const inliers, typename pcl::PointCloud<PointT>::Ptr const cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr const cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr const cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};

/* \author Aaron Brown */
// Quiz on implementing kd tree

// Structure to represent node of kd tree

template<typename PointT>
struct Node
{
    PointT point;
    int id;
    Node* left;
    Node* right;
    Node* top;

    Node(PointT PT, int setId)
    :	point(PT), id(setId), left(NULL), right(NULL),top(NULL)
    {}

    ~Node()
    {
        delete left;
        delete right;
        delete top;
    }
};
template<typename PointT>
class KdTree{

    Node<PointT> * root;

public:

    KdTree(): root(NULL){}
    ~KdTree(){ delete root; }

    void insertHelper(Node<PointT> ** node, int depth, PointT point, int id)
    {

        if(*node == NULL) *node = new Node<PointT>(point, id);
        else
        {
            uint cd = depth % 3;
            if(cd == 0){
                if(point.x < ((*node)->point.x))
                    insertHelper(&((*node)->left), depth+1, point, id);
                else
                    insertHelper(&((*node)->right), depth+1, point, id);
            }
            else if (cd == 1){
                if(point.y < ((*node)->point.y))
                    insertHelper(&((*node)->left), depth+1, point, id);
                else
                    insertHelper(&((*node)->right), depth+1, point, id);
            }

            else {
                if(point.z < ((*node)->point.z))
                    insertHelper(&((*node)->left), depth+1, point, id);
                else
                    insertHelper(&((*node)->right), depth+1, point, id);
            }
        }
    }

    void insert(PointT point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root

        insertHelper(&root, 0, point, id);

    }

    std::vector<int> searchHelper(Node<PointT> * node, int depth, const PointT target, float const distanceTol, std::vector<int> &ids)
    {

        if(node!=NULL)
        {
            if( (node->point.x >= (target.x-distanceTol) && node->point.x <= (target.x+distanceTol)) &&
                    (node->point.y >= (target.y-distanceTol) && node->point.y <= (target.y+distanceTol)) &&
                    (node->point.z >= (target.z-distanceTol) && node->point.z <= (target.z+distanceTol)) )

            {

                float d = sqrt((node->point.x - target.x)*(node->point.x - target.x) +
                               (node->point.y - target.y)*(node->point.y - target.y) +
                               (node->point.z - target.z)*(node->point.z - target.z) );

                if(d <= distanceTol)
                    ids.push_back(node->id);
            }

            // check cross boundary
            uint cd = depth % 3;


            if(cd == 0){
                if((target.x - distanceTol) < (node->point.x))
                    searchHelper(node->left, depth+1, target, distanceTol, ids);
                if((target.x + distanceTol) > (node->point.x))
                    searchHelper(node->right, depth+1, target, distanceTol, ids);
            }
            else if (cd == 1){
                if((target.y - distanceTol) < (node->point.y))
                    searchHelper(node->left, depth+1, target, distanceTol, ids);
                if((target.y + distanceTol) > (node->point.y))
                    searchHelper(node->right, depth+1, target, distanceTol, ids);
            }
            else {

                if((target.z - distanceTol) < (node->point.z))
                    searchHelper(node->left, depth+1, target, distanceTol, ids);
                if((target.z + distanceTol) > (node->point.z))
                    searchHelper(node->right, depth+1, target, distanceTol, ids);
             }
            }
            return ids;
        }



    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, 0, target, distanceTol, ids);
        return ids;
    }


};


#endif

