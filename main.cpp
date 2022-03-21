/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-04 15:38:03
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-03-21 13:45:16
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
    clock_t start_time, end_time;
    start_time = clock();


    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_none_nan(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_sparse(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_sta_filter(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_sta_filter_rotate(new pcl::PointCloud<PointT>);
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
    rotationTransformation(cloud_sta_filter, cloud_sta_filter_rotate, coefficients); // 旋转稀疏点云
    rotationTransformation(cloud_in, cloud_rotate, coefficients); // 旋转原始点云

    // 可视化点云
    // std::vector<pcl::PointCloud<PointT>::Ptr> clouds;
    // clouds.push_back(cloud_sta_filter);
    // clouds.push_back(cloud_rotate);
    // clouds.push_back(cloud_in);
    // multiCloudViewer(clouds);

    // 多平面分割
    std::vector<SegPlaneClouds> SegPlaneClouds_vtr;
    SegPlaneClouds_vtr = multiPlaneSegment(img_in, cloud_sta_filter_rotate, 3.0f, 1000, 0.2);
    for(int i = 0; i < SegPlaneClouds_vtr.size(); ++i)
    {
        std::string img_name = "../save_data/result" + std::to_string(i) + ".png";
        SegImages SegImages_;
        SegImages_ = pointCloud2Heightmap(img_in, cloud_rotate, SegPlaneClouds_vtr[i], 3.0f);

        cv::imwrite(img_name, SegImages_.gray_heightmap); // 保存图像
    }


    end_time = clock();
    std::cout << "Total time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
    
    return 0;
}