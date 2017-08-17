#include "perception.h"
#include <ctime>
#include <pcl/console/time.h>
#include <fstream>

int main(int argc, char** argv) {
	PointCloudPerception<ColoredPointT, ColoredPointTNormal> test;
	std::string test_file(argv[1]);// = "test_pcd_top.pcd";
	//pcl::PointCloud<ColoredPoinqtT>::Ptr cloud(new pcl::PointCloud<ColoredPointT>);
	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(
		new pcl::PointCloud<ColoredPointT>);
	//Eigen::Affine3f tf;
	//test.ApplyTransformToCombinedPointCloud(tf, cloud);
	test.VisualizePointCloudDrake(cloud);
	return 0;
}