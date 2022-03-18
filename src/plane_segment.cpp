/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-09 11:47:55
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-02-26 18:09:29
 * @Description:
 */

#include "plane_segment.h"

/**
 * @brief: 点云单平面分割
 * @param {Ptr} &cloud
 * @param {double} distance_threshold: 点到拟合平面的最大距离
 * @param {int} max_iter: 最大迭代次数 default = 1000
 * @param {Ptr} &remain: 剩余点云
 * @param {Ptr} &plane: 分割出的平面点云
 * @param {Ptr} &coefficients: 拟合得到的平面方程
 * @param {Ptr} &inliers: 平面点云在输入点云中的索引
 * @return {*}
 * @note:
 * @warning:
 */
bool planeSegment(pcl::PointCloud<PointT>::Ptr &cloud, double distance_threshold, int max_iter, pcl::PointCloud<PointT>::Ptr &remain, pcl::PointCloud<PointT>::Ptr &plane, pcl::ModelCoefficients::Ptr &coefficients, pcl::PointIndices::Ptr &inliers)
{
    pcl::SACSegmentation<PointT> seg; // 创建一个分割器

    // 修改一些分割器的参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold); // 点到拟合平面的最大距离
    seg.setMaxIterations(max_iter);
    seg.setInputCloud(cloud);             // 设置输入点云
    seg.segment(*inliers, *coefficients); // 开始分割，结果保存在两个对象中

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return false;
    }

    // 从输入点云中取出平面点云
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // 得到与平面点云相关的点
    extract.filter(*plane);

    // 从点云中剔除这些平面内点
    extract.setNegative(true);
    extract.filter(*remain);

    return true;
}

/**
 * @brief: 点云多平面分割
 * @param {Mat} &img_in: 输入图像
 * @param {Ptr} &cloud_in: 输入预处理后的点云
 * @param {double} distance_threshold: 平面分割距离阈值
 * @param {int} max_iter: 最大迭代次数 default = 1000
 * @param {float} stop_req: 分割停止条件，剩余点云占原始点云的百分比 default = 0.2
 * @return {*} 平面点云集合
 * @note:
 * @warning:
 */
std::vector<pcl::PointCloud<PointT>::Ptr> multiPlaneSegment(cv::Mat &img_in, pcl::PointCloud<PointT>::Ptr &cloud_in, double distance_threshold, int max_iter, float stop_req)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud_in, *cloud); // 深拷贝
    
    pcl::PointCloud<PointT>::Ptr cloud_seg_remain(new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 创建平面参数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    std::vector<pcl::PointCloud<PointT>::Ptr> planes;

    int point_num = cloud->size();
    int plane_num = 0;                              // 记录平面总数
    while (cloud->size() > stop_req * point_num) // 点云数不足 20% 时停止分割
    {
        pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT>); // 每次new一块新的内存，若放在循环外面，存入vector的指针指向同一块内存

        if (planeSegment(cloud, distance_threshold, max_iter, cloud_seg_remain, cloud_seg, coefficients, inliers) == false) // 内点为 0 时跳出循环
            break;

        planes.push_back(cloud_seg);
        std::cout << cloud->size() << std::endl;
        *cloud = *cloud_seg_remain;
        plane_num++;
    }
    std::cout << cloud_in->size() << std::endl;
    return planes;
}



/**
 * @brief: KD 树半径搜索
 * @param {PointCloud<PointT>::Ptr} &cloud 输入点云
 * @param {std::vector<pcl::PointCloud<PointT>::Ptr>} &search_clouds 要搜索的多个点云
 * @param {float} radius 搜索半径
 * @return {*}
 * @note: 
 * @warning: 
 */
// void kdtreeSearch(pcl::PointCloud<PointT>::Ptr &cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &search_clouds, float radius)
// {
//     pcl::KdTreeFLANN<PointT> kdtree;
//     kdtree.setInputCloud(cloud);

//     std::vector<std::vector<int>> pointIdxRadiusSearchs[search_clouds.size()];
//     std::vector<std::vector<float>> pointRadiusSquaredDistances[search_clouds.size()];

//     for (int i = 0; i < search_clouds.size(); i++) // 遍历点云容器
//     {
//         for (int j = 0; j < search_clouds[i]->size(); j++) // 遍历一个点云中每个点
//         {
//             std::vector<int> pointIdxRadiusSearch;
//             std::vector<float> pointRadiusSquaredDistance;
//             if (kdtree.radiusSearch(search_clouds[i]->points[j], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//             {
//                 pointIdxRadiusSearchs[i].insert(pointIdxRadiusSearchs[i].end(), pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
//                 // TO DO: 取并集，避免一个点在不同平面中重复
//             }
//         }
//     }
// }

// void octreeSearch(pcl::PointCloud<PointT>::Ptr &cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &search_cloud, float resolution)
// {
// }