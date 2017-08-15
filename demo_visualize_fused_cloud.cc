#include "perception.h"
#include <ctime>
#include <pcl/console/time.h>
#include <fstream>

int main(int argc, char** argv) {
	//GeometryAlignmentRepresentation<ColoredPointTNormal> point_representation;
	pcl::console::TicToc tt;

	PointCloudPerception<ColoredPointT, ColoredPointTNormal> test;
	std::string test_file(argv[1]);// = "test_pcd_top.pcd";
	//pcl::PointCloud<ColoredPoinqtT>::Ptr cloud(new pcl::PointCloud<ColoredPointT>);
	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(
		new pcl::PointCloud<ColoredPointT>);

	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals(
		new pcl::PointCloud<pcl::Normal>);


	boost::shared_ptr<pcl::PointCloud<ColoredPointTNormal>> cloud_and_normal(
		new pcl::PointCloud<ColoredPointTNormal>);
	pcl::PCDReader reader;
	reader.read<ColoredPointTNormal>(test_file, *cloud_and_normal);

	// tmp tf for MoveJ 53.741  33.0757 -6.67238 -38.0249 -29.5578  118.952 -2.52224 5
	Eigen::Affine3f tf_tmp;
	tf_tmp.matrix() << -0.799218 , 0.516049,  0.272282,  0.463091,
 			  0.352113,  0.817477, -0.444457,  0.449454,
			 -0.487101, -0.255781, -0.853417,  0.658012,
        		0,         0,         0,         1;
    test.ApplyTransformToCombinedPointCloud(tf_tmp, cloud_and_normal);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	Eigen::Vector4d coeffs_plane;
	test.FindPlane(cloud_and_normal, &coeffs_plane, inliers, 0.005);
	std::cout << coeffs_plane << std::endl;
	Eigen::Vector3f min_range;
	min_range << 0.45, -0.2, -0.2;
	Eigen::Vector3f max_range;
	max_range << 0.9, 0.2, 0.5;
	test.CutWithWorkSpaceConstraints(cloud_and_normal, min_range, max_range);

  	// Get rid of the table.
  	double thickness = 0.005;
  	test.SubtractTable(cloud_and_normal, thickness);

	
	test.FilterPointsWithEmptyNormals(cloud_and_normal);
	
	test.SeparatePointsAndNormals(cloud_and_normal, cloud, normals);
	
	Eigen::Vector3f top_left_corner, lower_right_corner;
	Eigen::Vector3f center;
	double coverage_ratio = atof(argv[2]);
	test.FindBoundingBox(cloud, &center, &top_left_corner, &lower_right_corner, coverage_ratio);
	std::cout << top_left_corner << std::endl;
	std::cout << lower_right_corner << std::endl;
	std::cout << center << std::endl;
	
	test.VisualizePointCloudAndNormal(cloud, normals);
	
	std::string normal_csv_file = "normals.csv";

	std::ofstream csv_output(normal_csv_file);
	for (int d = 0; d < 3; ++d) {
		for (int i = 0; i < normals->size(); ++i) {
			csv_output << normals->points[i].normal[d];
			if (i == normals->size() - 1) {
				csv_output << std::endl;
			} else {
				csv_output << ",";
			}
		}
	}
	csv_output.close();
	pcl::io::savePCDFileASCII("cloud.pcd", *cloud);

//	test.LoadPCDFile(test_file, cloud);
	//test.OutlierRemoval(cloud);

	//test.DownSample(cloud);
//	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals(
//		new pcl::PointCloud<pcl::Normal>);

//   tt.tic ();
 //	test.EstimateNormal(cloud, normals);
//	test.VisualizePointCloudAndNormal(cloud, normals);

	// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Eigen::Vector4d plane_coeffs;
	// test.FindPlane(cloud, &plane_coeffs, inliers);
	// std::cout << plane_coeffs.transpose() << std::endl;


  //test.VisualizePointCloudAndNormal(cloud, normals);
	// std::cout << "Input 6 dof camera" << std::endl;
	// Eigen::VectorXd camera_pose(6);
	// for (int i = 0; i < 6; ++i) {
	// 	cin >> camera_pose(i);
	// }

	// Eigen::Matrix3f M_rot = (Eigen::AngleAxisf(camera_pose(3) / 180.0 * M_PI, Eigen::Vector3f::UnitX())
 //    * Eigen::AngleAxisf(camera_pose(4) / 180.0 * M_PI, Eigen::Vector3f::UnitY())
	// 	* Eigen::AngleAxisf(camera_pose(5) / 180.0 * M_PI, Eigen::Vector3f::UnitZ())).toRotationMatrix();
	// Camera camera;
	// camera.pose(0,3) = camera_pose(0);
	// camera.pose(1,3) = camera_pose(1);
	// camera.pose(2,3) = camera_pose(2);
	// // camera.pose.translation() = camera_pose.head(3);
	// camera.pose.linear() = M_rot;

	// camera.fx = 570;
	// camera.fy = 570;
	// camera.img_height = 512;
	// camera.img_width = 512;
	// test.ApplyTransformToPointCloud(camera.pose.inverse(), cloud);

	// std::cout << "Input stride size for image completion" << std::endl;
	// int stride;
	// std::cin >> stride;
	// test.ProjectColoredPointCloudToCameraImagePlane(cloud, camera), stride;

	return 0;
}