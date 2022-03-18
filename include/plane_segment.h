/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-09 11:47:48
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-02-17 10:12:57
 * @Description:
 */

#pragma once

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h> // SAC 分割器
#include <pcl/kdtree/kdtree_flann.h>           // kdtree 搜索
#include <pcl/octree/octree_search.h>          // octree 搜索
#include <opencv2/opencv.hpp>
#include "template.h"

bool planeSegment(pcl::PointCloud<PointT>::Ptr &cloud, double distance_threshold, int max_iter, pcl::PointCloud<PointT>::Ptr &remain, pcl::PointCloud<PointT>::Ptr &plane, pcl::ModelCoefficients::Ptr &coefficients, pcl::PointIndices::Ptr &inliers);
std::vector<pcl::PointCloud<PointT>::Ptr> multiPlaneSegment(cv::Mat &img_in, pcl::PointCloud<PointT>::Ptr &cloud_in, double distance_threshold, int max_iter, float stop_req);
void kdtreeSearch(pcl::PointCloud<PointT>::Ptr &cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &search_clouds, float radius);