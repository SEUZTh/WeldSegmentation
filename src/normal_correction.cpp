/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-04 16:52:44
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-02-17 10:11:37
 * @Description:
 */

#include "normal_correction.h"

/**
 * @brief: 拟合平面方程
 * @param {Ptr} &cloud
 * @param {Ptr} &cloud_plane
 * @param {Ptr} inliers
 * @param {double} threshold default = 1
 * @param {int} max_iterations default = 1000
 * @return {Ptr} coefficients
 * @note:
 * @warning:
 */
pcl::ModelCoefficients::Ptr ransacFitting(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_plane, pcl::PointIndices::Ptr inliers, double threshold, int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 创建平面参数
    pcl::SACSegmentation<PointT> seg;                                     // 创建一个分割器
    // 修改一些分割器的参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(threshold); // 点到拟合平面的最大距离
    seg.setMaxIterations(max_iterations);
    seg.setInputCloud(cloud);                                            // 设置输入点云
    seg.segment(*inliers, *coefficients);                                // 开始分割，结果保存在两个对象中
    pcl::copyPointCloud<PointT>(*cloud, inliers->indices, *cloud_plane); // 根据索引提取内点

    return coefficients;
}

/**
 * @brief: 对点云的 pitch 和 roll 进行旋转
 * @param {Ptr} &cloud
 * @param {Ptr} &final
 * @param {Ptr} &coefficients
 * @return {*}
 * @note: 
 * @warning: 
 */
void rotationTransformation(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &final, pcl::ModelCoefficients::Ptr &coefficients)
{
    float pitch = atan(-coefficients->values[0] / coefficients->values[2]); // pitch = -arctan(-A/C)
    float roll = -atan(-coefficients->values[1] / coefficients->values[2]); // roll = -arctan(-B/C)

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud(*cloud, *final, transform);
}