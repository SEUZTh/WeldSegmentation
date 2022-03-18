/*
 * @Author: Tianhao Zhang
 * @Date: 2022-01-04 15:46:32
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-02-17 10:24:27
 * @Description: Please input description
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include "template.h"

void rm_nan(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &final);
void sparse(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, float leaf_size);
void staticFilter(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, int nr_k, double stddev_mult);
