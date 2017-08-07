#include "perception.h"
#include <ctime>
#include <pcl/console/time.h>

template <typename T>
void LoadPCDFile(std::string file_name, 
	boost::shared_ptr<pcl::PointCloud<T>> cloud) {
  pcl::PCDReader reader;
  reader.read<T>(file_name, *cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
}

template <typename T>
void PointCloudPerception::DownSample(boost::shared_ptr<pcl::PointCloud<T>> cloud, 
																			double leaf_size) {
	pcl::VoxelGrid<T> grid;
	grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);
}

template <typename T>
void PointCloudPerception::VisualizePointCloud(
    const boost::shared_ptr<pcl::PointCloud<T>>cloud, Eigen::Affine3f tf) {
  pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
	viewer.addCoordinateSystem(1.0, tf);
	pcl::visualization::PointCloudColorHandlerRGBField<ColoredPointT> rgb(cloud);
	viewer.addPointCloud(cloud, rgb, "Cloud");
	// Display the point cloud until 'q' key is pressed.
  while (!viewer.wasStopped ()) { 
    viewer.spinOnce ();
  }
}

template <typename T>
void PointCloudPerception::VisualizePointCloudAndNormal(
		const boost::shared_ptr<pcl::PointCloud<T>> cloud,
    boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals, 
    Eigen::Affine3f tf) {
	pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
	viewer.addCoordinateSystem(1.0, tf);
	pcl::visualization::PointCloudColorHandlerRGBField<ColoredPointT> rgb(cloud);
	viewer.addPointCloud(cloud, rgb, "Cloud");

	viewer.addPointCloudNormals<T,pcl::Normal>(cloud, normals);
	// Display the point cloud until 'q' key is pressed.
  while (!viewer.wasStopped ()) { 
    viewer.spinOnce ();
  }	
}


template <typename T> 
void PointCloudPerception::ApplyTransformToPointCloud(Eigen::Affine3f tf, 
		boost::shared_ptr<pcl::PointCloud<T>> cloud) {
	pcl::transformPointCloud(*cloud, *cloud, tf);		
}

template <typename T>
void PointCloudPerception::OutlierRemoval(
		boost::shared_ptr<pcl::PointCloud<T>> cloud, int num_neighbor, 
	double std_dev_threshold) {
  pcl::StatisticalOutlierRemoval<ColoredPointT> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (num_neighbor);
  sor.setStddevMulThresh (std_dev_threshold);
  sor.filter (*cloud);
}

template <typename T>
void PointCloudPerception::FindPlane(
		const boost::shared_ptr<pcl::PointCloud<T>> cloud, 
  	Eigen::Vector4d* plane_coefficients, pcl::PointIndices::Ptr inliers, 
  	double dist_threshold) {

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<ColoredPointT> seg;
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

template <typename T>
void PointCloudPerception::EstimateNormal(
		const boost::shared_ptr<pcl::PointCloud<T>> cloud, 
		boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals, double radius) {
	// Assume the camera is at the world origin.
	if (cloud->isOrganized()) {
		std::cout << "Use integral image normal esimation" << std::endl;
   	pcl::IntegralImageNormalEstimation<T, pcl::Normal> ne;
   	ne.setViewPoint(0.0, 0.0, 0.0);
    ne.setNormalEstimationMethod(
    	pcl::IntegralImageNormalEstimation<T, pcl::Normal>::COVARIANCE_MATRIX);
    ne.setInputCloud(cloud);
    ne.setRectSize(10, 10);
    ne.compute(*normals);
	} else {
		// Todo(jiaji): try out gpu based normal estimation.
		std::cout << "Use omp plain covariance-based normal estimation" << std::endl;
    pcl::NormalEstimationOMP<T, pcl::Normal> ne;
    ne.setViewPoint(0.0, 0.0, 0.0);
    ne.setInputCloud (cloud);
    //boost::shared_ptr<pcl::KdTreeFLANN<T>> tree(new pcl::KdTreeFLANN<T>());
    boost::shared_ptr<pcl::search::KdTree<T>> tree(new pcl::search::KdTree<T> ());
  	ne.setSearchMethod (tree);
  	// Set only one between the number of nearest neighbor and radius. 
    //ne.setKSearch(10);   
    ne.setRadiusSearch (radius);
    ne.compute(*normals);
	}
	// Compute the right direction of normals assuming camera is at the world 
	// origin and is viewing towards the positive z direction. 
	// for (int i = 0; i < cloud.size(); ++i) {
	// 	flipNormalTowardsViewpoint(const PointT &point, 0, 0, 0, Eigen::Vector4f &normal);
	// }
}


template <typename T, typename T2>
void PointCloudPerception::FusePointCloudPair(
		const boost::shared_ptr<pcl::PointCloud<T>> src,  
    const boost::shared_ptr<pcl::PointCloud<T>> tgt, 
    boost::shared_ptr<pcl::PointCloud<T2>> combined, 
    Eigen::Matrix4f* transform) {
	// First, estimate normal and curvture for each point cloud.
	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals_src(
	new pcl::PointCloud<pcl::Normal>);
	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals_tgt(
	new pcl::PointCloud<pcl::Normal>);
	double radius_normal_est = 0.02;
	EstimateNormal(src, normals_src, radius_normal_est);
	EstimateNormal(tgt, normals_tgt, radius_normal_est);
	boost::shared_ptr<pcl::PointCloud<T2>> cloud_with_normal_src(
		new pcl::PointCloud<T2>); 
	boost::shared_ptr<pcl::PointCloud<T2>> cloud_with_normal_tgt(
		new pcl::PointCloud<T2>); 
	pcl::concatenatePointCloud(src, normals_src, cloud_with_normal_src);
	pcl::concatenatePointCloud(tgt, normals_tgt, cloud_with_normal_tgt);
	PointCloudPairRegistration reg;
	reg.RegisterPointCloudPair(cloud_with_normal_src, cloud_with_normal_tgt,
															combined, transform);

}


template <typename T, typename T2>
void PointCloudPerception::FuseMultiPointClouds(
    const std::vector< boost::shared_ptr<pcl::PointCloud<T>> > point_clouds,
    boost::shared_ptr<pcl::PointCloud<T2>> combined_cloud) {
	for (unsigned i = 1; i < point_clouds.size(); ++i) {
		boost::shared_ptr<pcl::PointCloud<T2>> tmp_combined_cloud(new pcl::PointCloud<T2>);
		FuseMultiPointClouds<T, T2>(point_clouds[0], point_clouds[i], tmp_combined_cloud);
		*combined_cloud += * tmp_combined_cloud;
	}
}



cv::Mat PointCloudPerception::ProjectColoredPointCloudToCameraImagePlane(
	  const boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud, const Camera camera,
	  int stride) {
	// OpenCV is row-major allocation.
	cv::Mat projected_cv_img(camera.img_height, camera.img_width, CV_8UC3);
	int num_points = cloud->size();
	// Image coordinate (inferred from pcl openni2_grabber.cpp convertToXYZRGBPointCloud): 
	// top left corner is the origin. +x is along the increasing column direction. 
	// +y is along the increasing row direction.
	// Todo (Jiaji): it seems to be different from the openni kinect convention.
	// Check what's really going on. 

	// Get the image frame center.
	double cx = camera.img_width / 2.0;
	double cy = camera.img_height / 2.0;
	int K = camera.img_width * camera.img_height;
	bool mark[K];
	for (int i = 0; i < camera.img_width * camera.img_height; ++i){
		mark[i] = false;
	}
	for (int i = 0; i < num_points; ++i) {
		int row_id = round(cy + cloud->points[i].y / cloud->points[i].z * camera.fy);
		int col_id = round(cx + cloud->points[i].x / cloud->points[i].z * camera.fx);
		if (row_id > 0 && row_id < camera.img_height 
				&& col_id > 0 && col_id < camera.img_width) {
			// Assuming opencv is using bgr convention.
			// std::cout << row_id << "," << col_id << std::endl;
			int flag_id = row_id * camera.img_width + col_id;
			if (!mark[flag_id]) {
				projected_cv_img.at<cv::Vec3b>(row_id, col_id) = 
					cv::Vec3b(cloud->points[i].b, cloud->points[i].g, cloud->points[i].r);
				mark[flag_id] = true;
			}
		}
	}
	// Complete the holes due to missing point clouds in the object.
	for (int i = stride / 2; i < camera.img_height - stride / 2; ++i) { 
		for (int j = stride / 2; j < camera.img_width - stride / 2; ++j) {
			int flag_id = i * camera.img_width + j;
			if (!mark[flag_id]) {
				//std::cout << "r,c unmarked" << i << "," << j << std::endl;
				//std::cout << projected_cv_img.at<cv::Vec3b>(i,j) << std::endl;
				Eigen::Vector3d mean_color = Eigen::Vector3d::Zero();
				int num_averages = 0;
				for (int k1 = -stride / 2; k1 <= stride / 2; ++ k1) {
					for (int k2 = -stride / 2; k2 <= stride / 2; ++ k2) {
						int flag_neighbor_id = (i + k1) * camera.img_width + j + k2; 
						if (((k1 != 0) || (k2 != 0)) && mark[flag_neighbor_id]) {
							cv::Vec3b neighbor_color = projected_cv_img.at<cv::Vec3b>(i + k1, j + k2); 
							for (unsigned c = 0; c < 3; ++c) {
								mean_color(c) = mean_color(c) + neighbor_color[c];
							}
							++num_averages;
						}
					}
				}
				if (num_averages > 0) {
					//std::cout << mean_color << std::endl;
					mean_color = mean_color / num_averages;	
					//std::cout << mean_color << std::endl;
					projected_cv_img.at<cv::Vec3b>(i,j) = 
						cv::Vec3b(round(mean_color(0)), round(mean_color(1)), round(mean_color(2)));
				}
			}
		}
	}
	cv::namedWindow("test cv");
	//cv::Mat output_img;
	//cv::GaussianBlur(projected_cv_img, output_img, cv::Size(2,2), 0, 0);
	cv::imshow("cv_projected_image", projected_cv_img);
	cv::waitKey(0);
	return projected_cv_img;
}



int main(int argc, char** argv) {
	//GeometryAlignmentRepresentation<ColoredPointTNormal> point_representation;
	PointCloudPerception test;
	std::string test_file(argv[1]);// = "test_pcd_top.pcd";
	//pcl::PointCloud<ColoredPointT>::Ptr cloud(new pcl::PointCloud<ColoredPointT>);
	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(new pcl::PointCloud<ColoredPointT>);
	test.LoadPCDFile<ColoredPointT>(test_file, cloud);
	test.OutlierRemoval(cloud);
	test.VisualizePointCloud(cloud);
	// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Eigen::Vector4d plane_coeffs;
	// test.FindPlane(cloud, &plane_coeffs, inliers);
	// std::cout << plane_coeffs.transpose() << std::endl;

	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals(
		new pcl::PointCloud<pcl::Normal>);

	pcl::console::TicToc tt;
  tt.tic ();
	//test.EstimateNormal(cloud, normals);
	std::cout << tt.toc () / 1000.0 << std::endl;
  //test.VisualizePointCloudAndNormal(cloud, normals);
	std::cout << "Input 6 dof camera" << std::endl;
	Eigen::VectorXd camera_pose(6);
	for (int i = 0; i < 6; ++i) {
		cin >> camera_pose(i);
	}

	Eigen::Matrix3f M_rot = (Eigen::AngleAxisf(camera_pose(3) / 180.0 * M_PI, Eigen::Vector3f::UnitX()) 
    * Eigen::AngleAxisf(camera_pose(4) / 180.0 * M_PI, Eigen::Vector3f::UnitY()) 
		* Eigen::AngleAxisf(camera_pose(5) / 180.0 * M_PI, Eigen::Vector3f::UnitZ())).toRotationMatrix();
	Camera camera;
	camera.pose(0,3) = camera_pose(0);
	camera.pose(1,3) = camera_pose(1);
	camera.pose(2,3) = camera_pose(2);
	// camera.pose.translation() = camera_pose.head(3);
	camera.pose.linear() = M_rot;

	camera.fx = 570;
	camera.fy = 570;
	camera.img_height = 512;
	camera.img_width = 512;
	test.ApplyTransformToPointCloud(camera.pose.inverse(), cloud);
	
	std::cout << "Input stride size for image completion" << std::endl;
	int stride;
	std::cin >> stride;
	test.ProjectColoredPointCloudToCameraImagePlane(cloud, camera), stride;

	return 0;
}