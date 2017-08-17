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
	pcl::PCDReader reader;
	reader.read<ColoredPointT>(test_file, *cloud);
	Eigen::Affine3f tf;
	tf.matrix() <<
	0.121961,   0.977141,  -0.173951,   0.426154,
  0.980922, -0.0920011,   0.171295,  -0.101829,
  0.151325,  -0.191523,  -0.969772,   0.655674,
         0,          0,          0,          1;
	test.ApplyTransformToPointCloud(tf, cloud);
	test.VisualizePointCloudDrake(cloud);
	Eigen::Vector4d coeffs_plane;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	double dist_threshold = 0.01;
	test.FindPlane(cloud, &coeffs_plane, inliers, dist_threshold);
	std::cout << coeffs_plane << std::endl;
	return 0;
}