/*
 * @Auther: Tianhao Zhang
 * @Date: 2022-03-18 15:48:03
 * @LastEditors: Tianhao Zhang
 * @LastEditTime: 2022-03-20 16:31:16
 * @Description: 
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/fast_bilateral.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("../data/FeatureLocation2.pcd", *cloud) != 0)
	{
		return -1;
	}

	//------------------------双边滤波-----------------------
	pcl::FastBilateralFilter<pcl::PointXYZRGB> fbf;
	fbf.setInputCloud(cloud);
	fbf.setSigmaS(500);     // 设置双侧滤波器用于空间邻域/窗口的高斯的标准差。
	fbf.setSigmaR(3.0f); // 设置高斯的标准差，以控制由于强度差(在我们的情况下是深度)，相邻像素被降权的程度。

	fbf.filter(*cloud_filtered);

	//------------------------显示点云------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	viewer->setWindowName("快速双边滤波");
	int v1(0);
	// viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	// viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_filtered(cloud_filtered);
	// viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud", v1);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "cloud_filtered", v2);
	std::cout << cloud->size()<< std::endl;
	std::cout << cloud_filtered->size()<< std::endl;
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}
