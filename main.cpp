/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-04 15:38:03
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-02-26 18:05:55
 * @Description: 
 */
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include "template.h"
#include "pre_process.h"
#include "normal_correction.h"
#include "plane_segment.h"
#include "visualization.h"

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_none_nan(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_sparse(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_sta_filter(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_rotate(new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // 加载点云和图像
    pcl::io::loadPCDFile(argv[1],*cloud_in);
    cv::Mat img_in = cv::imread(argv[2], -1);

    // 预处理：移除Nan值，稀疏化
    rm_nan(cloud_in, cloud_none_nan);
    sparse(cloud_none_nan, cloud_sparse, 1.0f);
    staticFilter(cloud_sparse, cloud_sta_filter, 100, 1.0);
    
    // 法线纠正
    coefficients = ransacFitting(cloud_sta_filter, cloud_plane, inliers, 1.0, 1000);
    rotationTransformation(cloud_sta_filter, cloud_rotate, coefficients);

    // 可视化点云
    // std::vector<pcl::PointCloud<PointT>::Ptr> clouds;
    // clouds.push_back(cloud_sta_filter);
    // clouds.push_back(cloud_rotate);
    // clouds.push_back(cloud_in);
    // multiCloudViewer(clouds);

    // 多平面分割
    std::vector<pcl::PointCloud<PointT>::Ptr> planes;
    planes = multiPlaneSegment(img_in,cloud_sta_filter, 3.0f, 1000, 0.2);
    multiCloudViewer(planes);
    
    return 0;
}