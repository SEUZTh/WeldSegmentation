/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-04 15:59:52
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-02-17 10:23:55
 * @Description: 
 */

#include "pre_process.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

/**
 * @brief: 移除 Nan 值
 * @param {Ptr} &cloud
 * @param {Ptr} &final
 * @return {*}
 * @note: 
 * @warning: 
 */
void rm_nan(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &final)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *final, indices);
}

/**
 * @brief: 使用体素化网格方法对点云数据集进行下采样（即减少点数）
 * @param {Ptr} &cloud
 * @param {Ptr} &cloud_filtered
 * @param {float} leaf_size
 * @return {*}
 * @note: 
 * @warning: 
 */
void sparse(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, float leaf_size = 0.01)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setInputCloud(cloud);
    voxel.filter(*cloud_filtered);
    voxel.getRemovedIndices(*inliers);
}

// 统计滤波
/**
 * @brief 
 * 
 * @param cloud 
 * @param cloud_filtered 
 * @param nr_k default = 100 The number of points to use for mean distance estimation.
 * @param stddev_mult default = 1.0 The standard deviation multiplier.
 */
void staticFilter(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, int nr_k, double stddev_mult)
{
    pcl::StatisticalOutlierRemoval<PointT> stat;
    stat.setMeanK(nr_k);
    stat.setStddevMulThresh(stddev_mult);
    stat.setInputCloud(cloud);
    stat.setKeepOrganized(true);
    stat.filter(*cloud_filtered);
}