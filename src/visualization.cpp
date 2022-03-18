/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-09 12:37:32
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-02-26 17:36:40
 * @Description: 
 */

#include "visualization.h"

/**
 * @brief: 可视化多个点云
 * @param {vector<pcl::PointCloud<PointT>::Ptr>} clouds
 * @return {*}
 * @note: 
 * @warning: 
 */
void multiCloudViewer(std::vector<pcl::PointCloud<PointT>::Ptr> clouds)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Clouds Viewer"));

    for (int i = 0; i < clouds.size(); i++)
    {
        std::stringstream ss;
        ss << "cloud_" << i;
        pcl::visualization::PointCloudColorHandlerRandom<PointT> single_color(clouds[i]);
        viewer->addPointCloud<PointT>(clouds[i], single_color, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ss.str());
    }

    viewer->setBackgroundColor(0, 0, 0);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}