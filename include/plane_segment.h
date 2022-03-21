/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-09 11:47:48
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-03-21 12:47:15
 * @Description:
 */

#pragma once

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h> // SAC 分割器
#include <pcl/kdtree/kdtree_flann.h>           // kdtree 搜索
#include <pcl/octree/octree_search.h>          // octree 搜索
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "template.h"

struct SegImages {
    cv::Mat image;  // 分割得到的图像
    cv::Mat gray_heightmap; // 灰度heightmap
    cv::Mat color_heightmap;    // 彩色 heightmap
};

struct SegPlaneClouds {
    pcl::PointCloud<PointT>::Ptr plane_cloud;
    pcl::ModelCoefficients::Ptr plane_coefficient;
};

static bool compare_z(PointT a, PointT b)
{
    return (a.z < b.z);
}

bool planeSegment(pcl::PointCloud<PointT>::Ptr &cloud, double distance_threshold, int max_iter, pcl::PointCloud<PointT>::Ptr &remain, pcl::PointCloud<PointT>::Ptr &plane, pcl::ModelCoefficients::Ptr &coefficients, pcl::PointIndices::Ptr &inliers);
std::vector<pcl::PointCloud<PointT>::Ptr> multiPlaneCloudSegment(cv::Mat &img_in, pcl::PointCloud<PointT>::Ptr &cloud_in, double distance_threshold, int max_iter, float stop_req);
std::vector<SegPlaneClouds> multiPlaneSegment(cv::Mat &img_in, pcl::PointCloud<PointT>::Ptr &cloud_in, double distance_threshold, int max_iter, float stop_req);
double point2PlaneDistance(float x, float y, float z, pcl::ModelCoefficients::Ptr &coefficients);
SegImages pointCloud2Heightmap(cv::Mat &img_in, pcl::PointCloud<PointT>::Ptr &cloud_in, SegPlaneClouds SegPlaneCloud, double distance_threshold);
// void kdtreeSearch(pcl::PointCloud<PointT>::Ptr &cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &search_clouds, float radius);