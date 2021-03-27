#ifndef PCLFILE_H
#define PCLFILE_H

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <type_traits>
#include <boost/core/enable_if.hpp>
namespace Types {
enum PointType{
    XYZ,
    XYZI,
    Empty
};

}

template<typename PointT, typename Enable=void>
class FileManager{

public:
    void load(typename pcl::PointCloud<PointT>::Ptr  & cloud, std::string fileName){
        if(!fileName.empty())
        {
        if( *(fileName.rbegin()) == 't')
        {
            PointT point;
            std::ifstream inPCFile(fileName.c_str());
            // temporary string variable
            std::string line;

            while(!std::getline(inPCFile, line).eof())
            {
                  // temporary stream of string variable
                  std::stringstream sstream{line};
                  sstream>>point.x>>point.y>>point.z;
                  cloud->points.push_back(point);
            }
            cloud->width = cloud->points.size(); // number of data points in the set
            cloud->height = 1;                   // unorganised pcl
            cloud->is_dense = true;              // finite values
            inPCFile.close();
         }
        else
            {
             pcl::io::loadPCDFile<PointT>(fileName, *cloud);
            }
       }
    }

    void convert(typename pcl::PointCloud<PointT>::Ptr const & cloud, std::string fileName){
        if(!fileName.empty())
        {
        std::string newFileName = fileName;
        if( *(fileName.rbegin()) == 'd')
           {
              newFileName.replace(newFileName.size()-3, newFileName.size(), "txt");
              std::ofstream outPCFile(newFileName.c_str(), std::ios::app);
              for(auto point:*cloud)
                {
                    outPCFile<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
                }
              outPCFile.close();
           }
        else{
            newFileName.replace(newFileName.size()-3, newFileName.size(), "pcd");
            pcl::io::savePCDFileASCII (newFileName.c_str(), *cloud);
            }
        }
    }

    void save(typename pcl::PointCloud<PointT>::Ptr const & cloud, std::string fileName){
        if(!fileName.empty())
        {
        std::string newFileName = fileName;
        if( *(fileName.rbegin()) == 't')
           {
              newFileName.replace(newFileName.size()-4, newFileName.size(), "_new.txt");
              std::ofstream outPCFile(newFileName.c_str(), std::ios::app);
              for(auto point:*cloud)
                {
                    outPCFile<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
                }
              outPCFile.close();
           }
        else{
            newFileName.replace(newFileName.size()-4, newFileName.size(), "_new.pcd");
            pcl::io::savePCDFileASCII (newFileName.c_str(), *cloud);
            }
        }
    }



};

template<typename PointT>
class FileManager < PointT,  typename boost::enable_if<pcl::traits::has_field<PointT,pcl::fields::intensity>>::type>
{
public:
    void load(typename pcl::PointCloud<PointT>::Ptr  & cloud, std::string fileName){
        if(!fileName.empty())
        {
        if( *(fileName.rbegin()) == 't')
        {
            PointT point;
            std::ifstream inPCFile(fileName.c_str());
            // temporary string variable
            std::string line;

            while(!std::getline(inPCFile, line).eof())
            {
                  // temporary stream of string variable
                  std::stringstream sstream{line};
                  sstream>>point.x>>point.y>>point.z;
                  cloud->points.push_back(point);
            }
            cloud->width = cloud->points.size(); // number of data points in the set
            cloud->height = 1;                   // unorganised pcl
            cloud->is_dense = true;              // finite values
            inPCFile.close();
         }
        else
            {
             pcl::io::loadPCDFile<PointT>(fileName, *cloud);
            }
       }

    }
    void convert(typename pcl::PointCloud<PointT>::Ptr const & cloud, std::string fileName){
        if(!fileName.empty())
        {
        std::string newFileName = fileName;
        if( *(fileName.rbegin()) == 'd')
           {
              newFileName.replace(newFileName.size()-3, newFileName.size(), "txt");
              std::ofstream outPCFile(newFileName.c_str(), std::ios::app);
              for(auto point:*cloud)
                {
                    outPCFile<<point.x<<" "<<point.y<<" "<<point.z<<" "<<point.intensity<<std::endl;
                }
              outPCFile.close();
           }
        //else if(*(fileName.rbegin()) == 'd'){
        else{
            newFileName.replace(newFileName.size()-3, newFileName.size(), "pcd");
            pcl::io::savePCDFileASCII (newFileName.c_str(), *cloud);
            }
        }
    }

    void save(typename pcl::PointCloud<PointT>::Ptr const & cloud, std::string fileName){
        if(!fileName.empty())
        {
        std::string newFileName = fileName;
        if( *(fileName.rbegin()) == 't')
           {
              newFileName.replace(newFileName.size()-4, newFileName.size(), "_new.txt");
              std::ofstream outPCFile(newFileName.c_str(), std::ios::app);
              for(auto point:*cloud)
                {
                    outPCFile<<point.x<<" "<<point.y<<" "<<point.z<<" "<<point.intensity<<std::endl;
                }
              outPCFile.close();
           }
        //else if(*(fileName.rbegin()) == 'd'){
        else{
            newFileName.replace(newFileName.size()-4, newFileName.size(), "_new.pcd");
            pcl::io::savePCDFileASCII (newFileName.c_str(), *cloud);
            }
        }
    }

};

template <typename PointT>
class VoxelFilter{
    typename pcl::VoxelGrid<PointT> voxFilter;
    float voxelLeafSize;
public:
    VoxelFilter();
    ~VoxelFilter();
    void vFilter(typename pcl::PointCloud<PointT>::Ptr const& source, typename pcl::PointCloud<PointT>::Ptr const& processed);
    void set_voxelLeafSize(const float);

};

template <typename PointT>
class EculeadieanClustering{
    float distanceTolerance;
    unsigned int minClusterSize;
    unsigned int maxClusterSize;
    typename pcl::search::KdTree<PointT>::Ptr tree;
    typename pcl::EuclideanClusterExtraction<PointT> ec;

public:
    void set_distanceTolerance(const float);
    void set_minClusterSize(const unsigned int);
    void set_maxClusterSize(const unsigned int);
    void cluster(typename pcl::PointCloud<PointT>::Ptr const &,  std::vector<typename pcl::PointCloud<PointT>::Ptr>&);
    EculeadieanClustering();
    ~EculeadieanClustering();
};


template <typename PointT>
class ClusterManager{

public:
EculeadieanClustering<PointT> *ecClustering;
ClusterManager(){}
~ClusterManager(){}
};


template <typename PointT>
class FilterManager{

public:
VoxelFilter<PointT> *voxelFilter;
FilterManager(){}
~FilterManager(){}
};


template <typename PointT>
class PclProcessor{
public:
    PclProcessor();
    ~PclProcessor();
    FileManager<PointT> *fileManager;
    FilterManager<PointT> *filterManager;
    ClusterManager<PointT> *clusterManager;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
};

#endif // PCLFILE_H
