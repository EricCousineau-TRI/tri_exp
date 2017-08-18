#include "perception.h"
#include "openni_comm.h"
#include <ctime>
#include <pcl/console/time.h>
#include <fstream>

int main(int argc, char** argv) {
	PointCloudPerception<ColoredPointT, ColoredPointTNormal> test;
	//std::string test_file(argv[1]);// = "test_pcd_top.pcd";
	//pcl::PointCloud<ColoredPoinqtT>::Ptr cloud(new pcl::PointCloud<ColoredPointT>);
	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(
		new pcl::PointCloud<ColoredPointT>);
	OpenNiComm camera_interface;
	camera_interface.GetCurrentPointCloud(cloud);
	//pcl::PCDReader reader;
	//reader.read<ColoredPointT>(test_file, *cloud);
	
	Eigen::Affine3f tf;
	// tf.matrix() <<
	// 	0.184047,  0.966311, -0.179878,  0.405928,
 // 		0.970529,  -0.14973,  0.188919, -0.295445,
 // 		0.155592, -0.209368, -0.965381,  0.614409,
 //        0,         0,         0,         1;

  tf.matrix() << 
  	0.184553,  0.964496, -0.188871,   0.40626,
  	0.96953,   -0.1472,  0.195909, -0.295187,
 		0.161124, -0.219293, -0.962265,  0.610027,
        0,         0,         0,         1;

	test.ApplyTransformToPointCloud(tf, cloud);
	test.VisualizePointCloudDrake(cloud);
	Eigen::Vector4d coeffs_plane;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	double dist_threshold = 0.01;
	test.FindPlane(cloud, &coeffs_plane, inliers, dist_threshold);
	std::cout << coeffs_plane << std::endl;
	return 0;
}
