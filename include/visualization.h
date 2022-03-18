/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-01-09 12:36:47
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-01-09 12:38:14
 * @Description: 
 */

#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include "template.h"

void multiCloudViewer(std::vector<pcl::PointCloud<PointT>::Ptr> clouds);