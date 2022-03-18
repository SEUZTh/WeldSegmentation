/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-04 16:52:17
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-02-17 10:33:32
 * @Description: 
 */

#pragma once
#include "template.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>

pcl::ModelCoefficients::Ptr
ransacFitting(
    pcl::PointCloud<PointT>::Ptr &cloud,
    pcl::PointCloud<PointT>::Ptr &cloud_plane,
    pcl::PointIndices::Ptr inliers,
    double threshold,
    int max_iterations);

void 
rotationTransformation(
    pcl::PointCloud<PointT>::Ptr &cloud, 
    pcl::PointCloud<PointT>::Ptr &final, 
    pcl::ModelCoefficients::Ptr &coefficients);

