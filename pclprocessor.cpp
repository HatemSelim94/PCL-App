#include <pclprocessor.h>

struct delete_ptr {
    // Helper function to ease cleanup of container
    template <typename P>
    void operator () (P p) {
        p.reset();
    }
};



template<typename PointT>
PclProcessor<PointT>::PclProcessor()
{}

template<typename PointT>
PclProcessor<PointT>::~PclProcessor(){
    std::for_each(this->clusters.begin(), this->clusters.end(), delete_ptr());
    this->clusters.clear();
}

template<typename PointT>
EculeadieanClustering<PointT>::EculeadieanClustering():tree(new pcl::search::KdTree<PointT>), distanceTolerance(0.5), minClusterSize(20), maxClusterSize(3000) {}

template<typename PointT>
EculeadieanClustering<PointT>::~EculeadieanClustering(){tree.reset();}

template<typename PointT>
void EculeadieanClustering<PointT>::set_distanceTolerance(const float distanceTolerance)
{
   this->distanceTolerance = distanceTolerance;
}

template<typename PointT>
void EculeadieanClustering<PointT>::set_minClusterSize(const unsigned int minClusterSize)
{
    this->minClusterSize = minClusterSize;
}

template<typename PointT>
void EculeadieanClustering<PointT>::set_maxClusterSize(const unsigned int maxClusterSize)
{
    this->maxClusterSize = maxClusterSize;
}

template<typename PointT>
void EculeadieanClustering<PointT>::cluster(typename pcl::PointCloud<PointT>::Ptr const &cloud, std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters)
{

    std::vector<pcl::PointIndices> clusterIndices;
    this->tree->setInputCloud(cloud);
    this->ec.setClusterTolerance (this->distanceTolerance);                      //  tolerance is in m
    this->ec.setMinClusterSize (this->minClusterSize);
    this->ec.setMaxClusterSize (this->maxClusterSize);
    this->ec.setSearchMethod (this->tree);
    this->ec.setInputCloud (cloud);
    this->ec.extract (clusterIndices);
    for(auto Indices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new typename pcl::PointCloud<PointT>);
        for(auto index : Indices.indices)
            cloudCluster->push_back(cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }

}

template<typename PointT>
VoxelFilter<PointT>::VoxelFilter():voxelLeafSize(0.1){}

template<typename PointT>
VoxelFilter<PointT>::~VoxelFilter(){}

template<typename PointT>
void  VoxelFilter<PointT>::vFilter(typename pcl::PointCloud<PointT>::Ptr const & cloud, typename pcl::PointCloud<PointT>::Ptr const & processedCloud)
{
    this->voxFilter.setInputCloud (cloud);
    this->voxFilter.setLeafSize (this->voxelLeafSize, this->voxelLeafSize, this->voxelLeafSize);
    this->voxFilter.filter (*processedCloud);
}

template<typename PointT>
void VoxelFilter<PointT>::set_voxelLeafSize(const float leafSize)
{
    this->voxelLeafSize = leafSize;
}
