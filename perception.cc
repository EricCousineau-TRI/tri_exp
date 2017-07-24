#include "perception.h"

void PointCloudPerception::LoadColoredPCDFile(std::string file_name, 
		pcl::PointCloud<RGBD>::Ptr cloud) {
	pcl::PCDReader reader;
	reader.read<RGBD>(file_name, *cloud);
}

void PointCloudPerception::VisualizePointCloud(
		pcl::PointCloud<RGBD>::ConstPtr cloud, Eigen::Affine3f tf) {
	pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
	viewer.addCoordinateSystem(1.0, tf);
 	pcl::visualization::PointCloudColorHandlerRGBField<RGBD> rgb(cloud);
	viewer.addPointCloud(cloud, rgb, "Cloud");
	// Display the point cloud until 'q' key is pressed.
  while (!viewer.wasStopped ()) { 
    viewer.spinOnce ();
  }
}
 
void PointCloudPerception::ApplyTransformToPointCloud(Eigen::Affine3f tf, 
		pcl::PointCloud<RGBD>::Ptr  cloud) {
	pcl::transformPointCloud(*cloud, *cloud, tf);		
}

void PointCloudPerception::OutlierRemoval(
		pcl::PointCloud<RGBD>::Ptr cloud, int num_neighbor, 
	double std_dev_threshold) {
  pcl::StatisticalOutlierRemoval<RGBD> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (num_neighbor);
  sor.setStddevMulThresh (std_dev_threshold);
  sor.filter (*cloud);
}

void PointCloudPerception::SegmentPlane(
		const pcl::PointCloud<RGBD>::ConstPtr cloud, 
  	Eigen::Vector4d* plane_coefficients, pcl::PointIndices::Ptr inliers, 
  	double dist_threshold) {

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<RGBD> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(dist_threshold);  

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	//Eigen::Vector4d plane_coefficients;
	for (int i = 0; i < coefficients->values.size(); ++i) {
		(*plane_coefficients)(i) = coefficients->values[i];		
	}
}


int main() {
	PointCloudPerception test;
	std::string test_file = "test_pcd_top.pcd";
	pcl::PointCloud<RGBD>::Ptr cloud(new pcl::PointCloud<RGBD>);
	test.LoadColoredPCDFile(test_file, cloud);
	test.VisualizePointCloud(cloud);
	test.OutlierRemoval(cloud);
	test.VisualizePointCloud(cloud);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	Eigen::Vector4d plane_coeffs;
	test.SegmentPlane(cloud, &plane_coeffs, inliers);
	std::cout << plane_coeffs.transpose() << std::endl;
	return 0;
}