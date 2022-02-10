#include <Eigen/Core>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/correspondence_estimation.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
//#include "davidSDK/david.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> ColorHandlerT;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT1;
// This function displays the help
// This is the main function
int
main(int argc, char** argv)
{
	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointNormal>::Ptr original_data(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud_2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud_3(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud_4(new pcl::PointCloud<pcl::PointNormal>);

	pcl::io::loadPLYFile<pcl::PointNormal>("C:\\Users\\Ren\\Desktop\\點雲匹配測試檔\\B_Scan_1.ply", *original_data);

	//降點
	pcl::VoxelGrid<pcl::PointNormal> vg;
	const float leaf = 3.0f;
	vg.setInputCloud(original_data);
	vg.setLeafSize(leaf, leaf, leaf);
	vg.filter(*original_data);

	//计算点云质心
	Eigen::Vector4f cloudCentroid;
	pcl::compute3DCentroid(*original_data, cloudCentroid);

	//將點雲從質心移去PCL中心
	Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
	translation(0, 3) = -cloudCentroid[0];
	translation(1, 3) = -cloudCentroid[1];
	translation(2, 3) = -cloudCentroid[2];
	//cout << cloudCentroid.matrix() << endl;
	pcl::transformPointCloud(*original_data, *transformed_cloud, translation);

	//執行旋轉
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	float theta = M_PI / 4; // The angle of rotation in radians
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_2);

	//移回原本的位置
	translation(0, 3) = +cloudCentroid[0];
	translation(1, 3) = +cloudCentroid[1];
	translation(2, 3) = +cloudCentroid[2];
	pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, translation);

	//執行平移
	Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
	transform_3.translation() << 100.0f, 0.0, -40.0f;
	pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_3);
	pcl::io::savePLYFile("C:\\Users\\Ren\\Desktop\\點雲匹配測試檔\\B_Scan_1_tran.ply", *transformed_cloud);

	//視覺化
	pcl::visualization::PCLVisualizer visu51("Matrix transformation example");
	int v52(0);
	visu51.createViewPort(0.0, 0.0, 1.0, 1.0, v52);
	visu51.setBackgroundColor(0, 0, 0, v52);
	visu51.addText("Original Data", 15, 15, "v1 text", v52);
	visu51.addPointCloud(original_data, ColorHandlerT(original_data, 255.0, 255.0, 255.0), "original_data", v52);
	//visu51.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_data");
	visu51.addCoordinateSystem(50.0, "cloud", 0);
	visu51.addText("transformed_cloud", 15, 15, "v53 text", v52);
	visu51.addPointCloud(transformed_cloud, ColorHandlerT(transformed_cloud, 255.0, 0.0, 255.0), "transformed_cloud", v52);
	//visu51.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "transformed_cloud");
	visu51.spin();
	return 0;
}
