/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-09 11:47:55
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-03-21 14:27:54
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
std::vector<pcl::PointCloud<PointT>::Ptr> multiPlaneCloudSegment(cv::Mat &img_in, pcl::PointCloud<PointT>::Ptr &cloud_in, double distance_threshold, int max_iter, float stop_req)
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
 * @brief: 点云多平面分割
 * @param {Mat} &img_in
 * @param {Ptr} &cloud_in
 * @param {double} distance_threshold
 * @param {int} max_iter
 * @param {float} stop_req
 * @return {*} 结构体（平面电云、平面参数）
 * @note: 
 * @warning: 
 */
std::vector<SegPlaneClouds> multiPlaneSegment(cv::Mat &img_in, pcl::PointCloud<PointT>::Ptr &cloud_in, double distance_threshold, int max_iter, float stop_req)
{
    std::vector<SegPlaneClouds> SegPlaneClouds_vtr;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud_in, *cloud); // 深拷贝
    
    pcl::PointCloud<PointT>::Ptr cloud_seg_remain(new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    std::vector<pcl::PointCloud<PointT>::Ptr> planes;

    int point_num = cloud->size();
    int plane_num = 0;                              // 记录平面总数
    while (cloud->size() > stop_req * point_num) // 点云数不足 20% 时停止分割
    {
        SegPlaneClouds SegPlaneClouds;
        pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT>); // 每次new一块新的内存，若放在循环外面，存入vector的指针指向同一块内存
        pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);

        if (planeSegment(cloud, distance_threshold, max_iter, cloud_seg_remain, cloud_seg, coefficient, inliers) == false) // 内点为 0 时跳出循环
            break;

        SegPlaneClouds.plane_coefficient = coefficient;
        SegPlaneClouds.plane_cloud = cloud_seg;
        SegPlaneClouds_vtr.push_back(SegPlaneClouds);
        
        *cloud = *cloud_seg_remain;
        plane_num++;
    }
    return SegPlaneClouds_vtr;
}


/**
 * @brief: 计算点到平面的距离
 * @param {float} x
 * @param {float} y
 * @param {float} z
 * @param {Ptr} &coefficients
 * @return {*} 点到平面的距离
 * @note: 
 * @warning: 
 */
double point2PlaneDistance(float x, float y, float z, pcl::ModelCoefficients::Ptr &coefficients)
{
    return std::abs(x * coefficients->values[0] + y * coefficients->values[1] + z * coefficients->values[2] + coefficients->values[3]);
}


/**
 * @brief: 点云转深度图
 * @param {Ptr} &cloud_in
 * @param {Ptr} &cloud_seg
 * @param {Ptr} coefficients
 * @param {double} distance_threshold
 * @return {*}
 * @note: 
 * @warning: 
 */
SegImages pointCloud2Heightmap(cv::Mat &img_in, pcl::PointCloud<PointT>::Ptr &cloud_in, SegPlaneClouds SegPlaneCloud, double distance_threshold)
{
    SegImages SegImage1;
    cv::Mat heightmap = cv::Mat_<uchar>(cloud_in->height, cloud_in->width, CV_8UC1);
    cv::Mat img_seg = cv::Mat_<uchar>(cloud_in->height, cloud_in->width, CV_8UC1);

    float z = - SegPlaneCloud.plane_coefficient->values[3] / SegPlaneCloud.plane_coefficient->values[2];
    auto point_min_z = minmax_element(SegPlaneCloud.plane_cloud->points.begin(), SegPlaneCloud.plane_cloud->points.end(), compare_z); //返回迭代器
    float maxmin_distance = (*point_min_z.second).z - (*point_min_z.first).z;
    std::cout << maxmin_distance <<std::endl;

    for(int i = 0; i < cloud_in->height; i++)
    {
        for(int j = 0; j < cloud_in->width; j++)
        {
            int k = i * cloud_in->width + j;
            float d = point2PlaneDistance(cloud_in->points[k].x, cloud_in->points[k].y, cloud_in->points[k].z, SegPlaneCloud.plane_coefficient);
            // std::cout << "z: " << cloud_in->points[k].z - z << ", " << "d: " << d << std::endl;
            if(!std::isnan(cloud_in->points[k].z) && d < distance_threshold
                    // && cloud_in->points[k].z < max
                    // && cloud_in->points[k].z > min
            )
            {
                heightmap.at<uchar>(i, j) = (cloud_in->points[k].z - (*point_min_z.first).z) / maxmin_distance * 255.0f;
                img_seg.at<uchar>(i, j) = img_in.at<uchar>(i, j); // 将原图属于该平面的部分赋值给新图片
                cloud_in->points[k].z = -NAN;
                
            }
            else
            {
                heightmap.at<uchar>(i, j) = 255; // 非当前平面部分设为白色
                img_seg.at<uchar>(i, j) = 255; // 非当前平面部分设为白色
            }
            
        }
    }

    SegImage1.image = img_seg;
    SegImage1.gray_heightmap = heightmap;
    cv::applyColorMap(heightmap, SegImage1.color_heightmap, cv::COLORMAP_JET);

    return SegImage1;
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