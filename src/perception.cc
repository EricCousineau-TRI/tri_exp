#include "perception.h"
#include <ctime>
#include <pcl/console/time.h>
#include <pcl/common/pca.h>

#include <pcl/visualization/pcl_visualizer.h>

// Need to incorporate implementation.
#include <pcl/search/impl/search.hpp>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

#include <bot_core/pointcloud_t.hpp>

template <typename T, typename T2>
void PointCloudPerception<T, T2>::LoadPCDFile(std::string file_name,
		boost::shared_ptr<pcl::PointCloud<T>> cloud) {
  pcl::PCDReader reader;
  reader.read<T>(file_name, *cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
  std::cout << "remove Nan size" << indices.size() << std::endl;
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::LoadPCDFile(std::string file_name,
		boost::shared_ptr<pcl::PointCloud<T2>> cloud) {
  pcl::PCDReader reader;
  reader.read<T2>(file_name, *cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
  std::cout << "remove Nan size" << indices.size() << std::endl;
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::DownSample(boost::shared_ptr<pcl::PointCloud<T>> cloud,
		double leaf_size) {
	std::cout << "Number of points before down sampling" << cloud->size() << std::endl;
	pcl::VoxelGrid<T> grid;
	grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);
	std::cout << "Number of points after down sampling" << cloud->size() << std::endl;
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::DownSample(boost::shared_ptr<pcl::PointCloud<T2>> cloud,
		double leaf_size) {
	std::cout << "Number of points before down sampling" << cloud->size() << std::endl;
	pcl::VoxelGrid<T2> grid;
	grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);
	std::cout << "Number of points after down sampling" << cloud->size() << std::endl;
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::FilterPointsWithEmptyNormals(
		boost::shared_ptr<pcl::PointCloud<T2>> cloud) {

	int num_points = cloud->size();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	for (int i = 0; i < num_points; ++i) {
		bool has_normal = true;
		for (int j = 0; j < 3; ++j) {
			//std::cout << cloud->points[i].normal[j] << ",";
			// Elastic fusion can sometimes return normals having compoenent slightly
			// larger than 1.0.
			if (!((cloud->points[i].normal[j] >= - 2 && cloud->points[i].normal[j] <= 2))) {
				has_normal = false;
			}
		}
		//std::cout << std::endl;
		if (has_normal) {
			inliers->indices.push_back(i);
		}
	}
  pcl::ExtractIndices<T2> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  // Keep the inlier points;
  extract.setNegative (false);
  extract.filter (*cloud);
}


template <typename T, typename T2>
void PointCloudPerception<T, T2>::CutWithWorkSpaceConstraints(
		boost::shared_ptr<pcl::PointCloud<T>> cloud, 
		const Eigen::Vector3f & min_range, const Eigen::Vector3f& max_range) {
	int counter = 0;
	int num_original_size = cloud->size();
	std::cout << "Original size before ws cut " << num_original_size << std::endl;
	for (int i = 0; i < num_original_size; ++i) {
		if ((cloud->points[i].x >= min_range(0)) && (cloud->points[i].x <= max_range(0))
			&& (cloud->points[i].y >= min_range(1)) && (cloud->points[i].y <= max_range(1))
		  && (cloud->points[i].z >= min_range(2)) && (cloud->points[i].z <= max_range(2))) {
			cloud->points[counter] = cloud->points[i];
			counter++;
		}
	}
	cloud->resize(counter);
	std::cout << "After cut " << cloud->size() << std::endl;
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::CutWithWorkSpaceConstraints(
		boost::shared_ptr<pcl::PointCloud<T2>> cloud, 
		const Eigen::Vector3f & min_range, const Eigen::Vector3f& max_range) {
	int counter = 0;
	int num_original_size = cloud->size();
	std::cout << "Original size before ws cut " << num_original_size << std::endl;
	for (int i = 0; i < num_original_size; ++i) {
		if ((cloud->points[i].x >= min_range(0)) && (cloud->points[i].x <= max_range(0))
			&& (cloud->points[i].y >= min_range(1)) && (cloud->points[i].y <= max_range(1))
		  && (cloud->points[i].z >= min_range(2)) && (cloud->points[i].z <= max_range(2))) {
			cloud->points[counter] = cloud->points[i];
			counter++;
		}
	}
	cloud->resize(counter);
	std::cout << "After cut " << cloud->size() << std::endl;
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::VisualizePointCloud(
    const boost::shared_ptr<pcl::PointCloud<ColoredPointT>>cloud, Eigen::Affine3f tf) {
  // throw std::runtime_error("Not implemented");
	std::cout << "open up viewer" << std::endl;
  pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
	viewer.addCoordinateSystem(0.2, tf);
  pcl::visualization::PointCloudColorHandlerCustom<ColoredPointT> single_color (cloud, 0, 255, 0);
	// pcl::visualization::PointCloudColorHandlerRGBField<ColoredPointT> rgb(cloud);
	std::cout << "!!" << std::endl;
	// viewer.addPointCloud(cloud, rgb, "Cloud");
  viewer.addPointCloud(cloud, single_color, "Cloud");
	std::cout << "to display" << std::endl;
	// Display the point cloud until 'q' key is pressed.
  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }
}

template <typename PointT>
void VisualizePointCloudDrakeImpl(
    drake::lcm::DrakeLcm* lcm,
    const boost::shared_ptr<pcl::PointCloud<PointT>> cloud,
    Eigen::Isometry3d X_WC,
    const std::string& suffix) {
  bot_core::pointcloud_t message;
  message.points.clear();
  message.frame_id = "world";
  message.n_points = cloud->points.size();
  message.points.resize(message.n_points);
  // See: director.drakevisualizer, DrakeVisualier.onPointCloud
  message.n_channels = 3;
  message.channel_names = {"r", "g", "b"};
  message.channels.resize(3, std::vector<float>(message.n_points));
  for (int i = 0; i < message.n_points; ++i) {
    const auto& point = cloud->points[i];
    message.channels[0][i] = point.r / 255.0;
    message.channels[1][i] = point.g / 255.0;
    message.channels[2][i] = point.b / 255.0;
    Eigen::Vector3f pt_W =
        (X_WC * Eigen::Vector3d(point.x, point.y, point.z)).cast<float>();
    message.points[i] = {pt_W[0], pt_W[1], pt_W[2]};
  }
  message.n_points = message.points.size();
  std::vector<uint8_t> bytes(message.getEncodedSize());
  message.encode(bytes.data(), 0, bytes.size());
  lcm->Publish("DRAKE_POINTCLOUD_" + suffix, bytes.data(), bytes.size());
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::VisualizePointCloudDrake(
      const boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud,
      Eigen::Isometry3d X_WC,
      const std::string& suffix) {
  VisualizePointCloudDrakeImpl(&lcm_, cloud, X_WC, suffix);
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::VisualizePointCloudDrake(
      const boost::shared_ptr<pcl::PointCloud<ColoredPointTNormal>> cloud,
      Eigen::Isometry3d X_WC,
      const std::string& suffix) {
  VisualizePointCloudDrakeImpl(&lcm_, cloud, X_WC, suffix);
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::VisualizePointCloudAndNormal(
		const boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud,
    boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals,
    Eigen::Affine3f tf) {
  // throw std::runtime_error("Not implemented");
	pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
	viewer.addCoordinateSystem(0.2, tf);
	pcl::visualization::PointCloudColorHandlerRGBField<ColoredPointT> rgb(cloud);
	viewer.addPointCloud(cloud, rgb, "Cloud");

	viewer.addPointCloudNormals<ColoredPointT, pcl::Normal>(cloud, normals);
	// Display the point cloud until 'q' key is pressed.
  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::SeparatePointsAndNormals(
    const boost::shared_ptr<pcl::PointCloud<T2>> points_and_normal, 
    boost::shared_ptr<pcl::PointCloud<T>> points, 
    boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals) {
	pcl::copyPointCloud(*points_and_normal, *points);
	pcl::copyPointCloud(*points_and_normal, *normals);

}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::SubtractTable(
		boost::shared_ptr<pcl::PointCloud<T>> cloud, double table_thickness,
    boost::shared_ptr<pcl::PointCloud<T>> removed) {
	  // Get rid of the table.
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
  Eigen::Vector4d coeffs_plane;
  FindPlane(cloud, &coeffs_plane, inliers, table_thickness);
  int num_before = cloud->size();
  pcl::ExtractIndices<T> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);

  typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>());
  extract.filter (*cloud_filtered);

  extract.setNegative(false);
  extract.filter (*removed);

  cloud = cloud_filtered;

  // cloud.sawp(cloud_filtered;
  int num_after = cloud->size();
  std::cout << "Before: " << num_before
      << ", After: " << num_after << std::endl;
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::SubtractTable(
		boost::shared_ptr<pcl::PointCloud<T2>> cloud, double table_thickness) {
	  // Get rid of the table.
  throw std::runtime_error("Not implemented");
  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
  // Eigen::Vector4d coeffs_plane;
  // FindPlane(cloud, &coeffs_plan1e, inliers, table_thickness);
  // pcl::ExtractIndices<T2> extract;
  // extract.setInputCloud (cloud);
  // extract.setIndices (inliers);
  // extract.setNegative (true);
  // extract.filter (*cloud);
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::FilterPointsBasedOnScatterness(
		boost::shared_ptr<pcl::PointCloud<T2>> cloud, double cover_ratio) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	//std::vector<int> indices_near;
	SelectNearCentroidPoints(cloud, &(inliers->indices), cover_ratio);
	pcl::ExtractIndices<T2> extract;
  extract.setInputCloud (cloud);
  //inliers->indices = indices_near;
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud);	
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::SelectNearCentroidPoints(
		const boost::shared_ptr<pcl::PointCloud<T2>> cloud, 
		std::vector<int>* indices, double cover_ratio) {

	int num_pts = cloud->size();
	Eigen::Vector4f mid_pt;
	pcl::compute3DCentroid(*cloud, mid_pt);

	// Compute average distance to the center.
	double avg_dist = 0;
	std::vector<double> all_dists(num_pts);
	std::vector<double> all_dists_cp(num_pts);
	for (int i = 0; i < cloud->size(); ++i) {
		Eigen::Vector4f pt;
		pt[0] = float(cloud->points[i].x);
		pt[1] = float(cloud->points[i].y);
		pt[2] = float(cloud->points[i].z);
		pt[3] = 0; 
		all_dists[i] = (pt - mid_pt).norm();
		all_dists_cp[i] = (pt - mid_pt).norm();
		avg_dist = avg_dist + all_dists[i] ;
	}
	avg_dist = avg_dist / cloud->size();
	// Sort the distances and only keep the first 95% for computation of initial
	// bounding box. Update the middle point.
	sort(all_dists.begin(), all_dists.end());
	int index_threshold = std::min(int(floor(num_pts * cover_ratio)), num_pts - 1);
	double dist_threshold = all_dists[index_threshold];
	indices->clear();
	for (int i = 0; i < num_pts; ++i) {
		if (all_dists_cp[i] < dist_threshold) {
			indices->push_back(i);
		}
	}
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::FilterPointsBasedOnScatterness(
		boost::shared_ptr<pcl::PointCloud<T>> cloud, double cover_ratio) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	//std::vector<int> indices_near;
	SelectNearCentroidPoints(cloud, &(inliers->indices), cover_ratio);
	pcl::ExtractIndices<T> extract;
  extract.setInputCloud (cloud);
  //inliers->indices = indices_near;
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud);	
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::SelectNearCentroidPoints(
		const boost::shared_ptr<pcl::PointCloud<T>> cloud, 
		std::vector<int>* indices, double cover_ratio) {

	int num_pts = cloud->size();
	Eigen::Vector4f mid_pt;
	pcl::compute3DCentroid(*cloud, mid_pt);

	// Compute average distance to the center.
	double avg_dist = 0;
	std::vector<double> all_dists(num_pts);
	std::vector<double> all_dists_cp(num_pts);
	for (int i = 0; i < cloud->size(); ++i) {
		Eigen::Vector4f pt;
		pt[0] = float(cloud->points[i].x);
		pt[1] = float(cloud->points[i].y);
		pt[2] = float(cloud->points[i].z);
		pt[3] = 0; 
		all_dists[i] = (pt - mid_pt).norm();
		all_dists_cp[i] = (pt - mid_pt).norm();
		avg_dist = avg_dist + all_dists[i] ;
	}
	avg_dist = avg_dist / cloud->size();
	// Sort the distances and only keep the first 95% for computation of initial
	// bounding box. Update the middle point.
	sort(all_dists.begin(), all_dists.end());
	int index_threshold = std::min(int(floor(num_pts * cover_ratio)), num_pts - 1);
	double dist_threshold = all_dists[index_threshold];
	indices->clear();
	for (int i = 0; i < num_pts; ++i) {
		if (all_dists_cp[i] < dist_threshold) {
			indices->push_back(i);
		}
	}
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::FindBoundingBox(
		const boost::shared_ptr<pcl::PointCloud<T>> cloud,
		Eigen::Vector3f* center, Eigen::Vector3f* top_corner, 
		Eigen::Vector3f* lower_corner, Eigen::Matrix3f* orientation,
		double cover_ratio) {
	int num_pts = cloud->size();
	double left_scale = 0;
	double right_scale = 1.0;
	double eps_diff = 1e-4;
	// Eigen uses 4f for its interface.
	Eigen::Vector4f min_pt, max_pt;
	Eigen::Vector4f mid_pt;
	std::vector<int> remaining_index;
	SelectNearCentroidPoints(cloud, &remaining_index, cover_ratio);

	pcl::compute3DCentroid(*cloud, remaining_index, mid_pt);
	pcl::PCA<T> pca;
	pca.setInputCloud(cloud);
	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
	*orientation = eigen_vectors;
	eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

	std::cout << "PCAs:" << std::endl;
	std::cout << eigen_vectors << std::endl;
	boost::shared_ptr<pcl::PointCloud<T>> cloud_projected (new pcl::PointCloud<T>); 
	pca.project(*cloud, *cloud_projected);

	pcl::getMinMax3D(*cloud_projected, remaining_index, min_pt, max_pt);
	T pt_projected_min;
	T pt_projected_max;
	pt_projected_min.x = min_pt(0);
	pt_projected_min.y = min_pt(1);
	pt_projected_min.z = min_pt(2);
	pt_projected_max.x = max_pt(0);
	pt_projected_max.y = max_pt(1);
	pt_projected_max.z = max_pt(2);

	std::cout << pt_projected_min << std::endl;
	std::cout << pt_projected_max << std::endl;
	std::cout << "----------------" << std::endl;
	T pt_min, pt_max;
	pca.reconstruct(pt_projected_min, pt_min);
	pca.reconstruct(pt_projected_max, pt_max);
	std::cout << pt_min << std::endl;
	std::cout << pt_max << std::endl; 

	*top_corner = Eigen::Vector3f(pt_max.x,pt_max.y,pt_max.z);
	*lower_corner = Eigen::Vector3f(pt_min.x,pt_min.y,pt_min.z);
	//*center = Eigen::Vector3f(mid_pt(0),mid_pt(1),mid_pt(2));
	*center = (*top_corner + *lower_corner) / 2.0;
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::FindBoundingBox(
		const boost::shared_ptr<pcl::PointCloud<T2>> cloud,
		Eigen::Vector3f* center, Eigen::Vector3f* top_corner, 
		Eigen::Vector3f* lower_corner, Eigen::Matrix3f* orientation,
		double cover_ratio) {
	int num_pts = cloud->size();
	double left_scale = 0;
	double right_scale = 1.0;
	double eps_diff = 1e-4;
	// Eigen uses 4f for its interface.
	Eigen::Vector4f min_pt, max_pt;
	Eigen::Vector4f mid_pt;
	std::vector<int> remaining_index;
	SelectNearCentroidPoints(cloud, &remaining_index, cover_ratio);

	pcl::compute3DCentroid(*cloud, remaining_index, mid_pt);
	pcl::PCA<T2> pca;
	pca.setInputCloud(cloud);
	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
	*orientation = eigen_vectors;
	eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

	std::cout << "PCAs:" << std::endl;
	std::cout << eigen_vectors << std::endl;
	boost::shared_ptr<pcl::PointCloud<T2>> cloud_projected (new pcl::PointCloud<T2>); 
	pca.project(*cloud, *cloud_projected);

	pcl::getMinMax3D(*cloud_projected, remaining_index, min_pt, max_pt);
	T2 pt_projected_min;
	T2 pt_projected_max;
	pt_projected_min.x = min_pt(0);
	pt_projected_min.y = min_pt(1);
	pt_projected_min.z = min_pt(2);
	pt_projected_max.x = max_pt(0);
	pt_projected_max.y = max_pt(1);
	pt_projected_max.z = max_pt(2);

	std::cout << pt_projected_min << std::endl;
	std::cout << pt_projected_max << std::endl;
	std::cout << "----------------" << std::endl;
	T2 pt_min, pt_max;
	pca.reconstruct(pt_projected_min, pt_min);
	pca.reconstruct(pt_projected_max, pt_max);
	std::cout << pt_min << std::endl;
	std::cout << pt_max << std::endl; 

	*top_corner = Eigen::Vector3f(pt_max.x,pt_max.y,pt_max.z);
	*lower_corner = Eigen::Vector3f(pt_min.x,pt_min.y,pt_min.z);
	//*center = Eigen::Vector3f(mid_pt(0),mid_pt(1),mid_pt(2));
	*center = (*top_corner + *lower_corner) / 2.0;

}
   

template <typename T, typename T2>
void PointCloudPerception<T, T2>::ApplyTransformToPointCloud(Eigen::Affine3f tf,
		boost::shared_ptr<pcl::PointCloud<T>> cloud) {
	pcl::transformPointCloud(*cloud, *cloud, tf);
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::ApplyTransformToCombinedPointCloud(Eigen::Affine3f tf,
		boost::shared_ptr<pcl::PointCloud<T2>> cloud_and_normal) {
	boost::shared_ptr<pcl::PointCloud<T>> cloud(
		new pcl::PointCloud<T>);

	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals(
		new pcl::PointCloud<pcl::Normal>);

	SeparatePointsAndNormals(cloud_and_normal, cloud, normals);
	pcl::transformPointCloud(*cloud, *cloud, tf);
	Eigen::Affine3f tf_rot = tf;
	tf_rot.translation() = Eigen::Vector3f::Zero();
	//pcl::transformPointCloud(*normals, *normals, tf_rotation_only);
	for (int i = 0; i < normals->size(); ++i) {
  	Eigen::Vector3f normal_i((*normals)[i].normal_x, (*normals)[i].normal_y, 
  		(*normals)[i].normal_z);
  	normal_i = tf_rot * normal_i;
  	(*normals)[i].normal_x = normal_i(0);
  	(*normals)[i].normal_y = normal_i(1);
  	(*normals)[i].normal_z = normal_i(2);
  }
  cloud_and_normal->clear();
	pcl::concatenateFields(*cloud, *normals, *cloud_and_normal);
}


template <typename T, typename T2>
void PointCloudPerception<T, T2>::OutlierRemoval(
		boost::shared_ptr<pcl::PointCloud<T>> cloud, int num_neighbor,
	double std_dev_threshold) {
  pcl::StatisticalOutlierRemoval<T> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (num_neighbor);
  sor.setStddevMulThresh (std_dev_threshold);
  sor.filter (*cloud);
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::OutlierRemoval(
		boost::shared_ptr<pcl::PointCloud<T2>> cloud, int num_neighbor,
	double std_dev_threshold) {
  pcl::StatisticalOutlierRemoval<T2> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (num_neighbor);
  sor.setStddevMulThresh (std_dev_threshold);
  sor.filter (*cloud);
}

template <typename T, typename T2>
void PointCloudPerception<T, T2>::FindPlane(
		const boost::shared_ptr<pcl::PointCloud<T>> cloud,
  	Eigen::Vector4d* plane_coefficients, pcl::PointIndices::Ptr inliers,
  	double dist_threshold) {

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<T> seg;
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


template <typename T, typename T2>
void PointCloudPerception<T, T2>::FindPlane(
		const boost::shared_ptr<pcl::PointCloud<T2>> cloud,
  	Eigen::Vector4d* plane_coefficients, pcl::PointIndices::Ptr inliers,
  	double dist_threshold) {

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<T2> seg;
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

template <typename T, typename T2>
void PointCloudPerception<T, T2>::EstimateNormal(
		const boost::shared_ptr<pcl::PointCloud<T>> cloud,
		boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals, double radius) {
	// Assume the camera is at the world origin.
	// if (cloud->isOrganized()) {
	// 	std::cout << "Use integral image normal esimation" << std::endl;
 //   	pcl::IntegralImageNormalEstimation<T, pcl::Normal> ne;
 //   	ne.setViewPoint(0.0, 0.0, 0.0);
 //    ne.setNormalEstimationMethod(
 //    	pcl::IntegralImageNormalEstimation<T, pcl::Normal>::COVARIANCE_MATRIX);
 //    ne.setInputCloud(cloud);
 //    ne.setRectSize(10, 10);
 //    ne.compute(*normals);
	// } else {
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
	//}
	// Compute the right direction of normals assuming camera is at the world
	// origin and is viewing towards the positive z direction.
	for (int i = 0; i < cloud->size(); ++i) {
		double x,y,z,nx,ny,nz;
		x = cloud->points[i].x;
		y = cloud->points[i].y;
		z = cloud->points[i].z;
		nx = normals->points[i].normal_x;
		ny = normals->points[i].normal_y;
		nz = normals->points[i].normal_z;
		if (x * nx + y * ny + z * nz > 0) {
			normals->points[i].normal_x = -nx;
			normals->points[i].normal_y = -ny;
			normals->points[i].normal_z = -nz;
		}
	}
}

// template <typename T>
// template <typename T2>
// void PointCloudPerception<T>::FusePointCloudPair(
// 		const boost::shared_ptr<pcl::PointCloud<T>> src,
//     const boost::shared_ptr<pcl::PointCloud<T>> tgt,
//     boost::shared_ptr<pcl::PointCloud<T2>> combined,
//     Eigen::Matrix4f* transform) {
// 	// First, estimate normal and curvture for each point cloud.
// 	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals_src(
// 			new pcl::PointCloud<pcl::Normal>);
// 	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals_tgt(
// 			new pcl::PointCloud<pcl::Normal>);
// 	double radius_normal_est = 0.02;
// 	EstimateNormal(src, normals_src, radius_normal_est);
// 	EstimateNormal(tgt, normals_tgt, radius_normal_est);
// 	boost::shared_ptr<pcl::PointCloud<T2>> cloud_with_normal_src(
// 		new pcl::PointCloud<T2>);
// 	boost::shared_ptr<pcl::PointCloud<T2>> cloud_with_normal_tgt(
// 		new pcl::PointCloud<T2>);

// 	const pcl::PointCloud<T> & src_alias = *src;
// 	const pcl::PointCloud<pcl::Normal> & normals_src_alias = *normals_src;

// 	pcl::concatenateFields<T, pcl::Normal, T2>(*src, *normals_src, *cloud_with_normal_src);
// 	pcl::concatenateFields<T, pcl::Normal, T2>(*tgt, *normals_tgt, *cloud_with_normal_tgt);
// 	PointCloudPairRegistration reg;
// 	reg.RegisterPointCloudPair<T2>(cloud_with_normal_src, cloud_with_normal_tgt,
// 																combined, transform);
// }

// template <typename T>
// template <typename T2>
// void PointCloudPerception<T>::FuseMultiPointClouds(
//     const std::vector< boost::shared_ptr<pcl::PointCloud<T>> > point_clouds,
//     boost::shared_ptr<pcl::PointCloud<T2>> combined_cloud) {
// 	Eigen::Affine3f global_tf_affine = Eigen::Affine3f::Identity();
// 	//Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
// 	for (unsigned i = 1; i < point_clouds.size(); ++i) {
// 		boost::shared_ptr<pcl::PointCloud<T2>> tmp_combined_cloud(new pcl::PointCloud<T2>);
// 		Eigen::Matrix4f relative_transform;
// 		FusePointCloudPair<T2>(point_clouds[i-1], point_clouds[i],
// 				tmp_combined_cloud, &relative_transform);
// 		global_tf_affine.matrix() = global_tf_affine.matrix() * relative_transform;
// 		ApplyTransformToPointCloud<T2>(global_tf_affine, tmp_combined_cloud);
// 		*combined_cloud += *tmp_combined_cloud;
// 	}
// }

#if false
template <typename T, typename T2>
cv::Mat PointCloudPerception<T, T2>::ProjectColoredPointCloudToCameraImagePlane(
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
#endif  // false

template class PointCloudPerception<PointT, PointTNormal>;
template class PointCloudPerception<ColoredPointT, ColoredPointTNormal>;

// template void PointCloudPerception<PointT>::ApplyTransformToPointCloud(Eigen::Affine3f,
// 		boost::shared_ptr<pcl::PointCloud<PointT>>);

// template void PointCloudPerception<PointT>::ApplyTransformToPointCloud(Eigen::Affine3f,
// 		boost::shared_ptr<pcl::PointCloud<PointTNormal>>);

// template void PointCloudPerception<ColoredPointT>::ApplyTransformToPointCloud(Eigen::Affine3f,
// 		boost::shared_ptr<pcl::PointCloud<ColoredPointT>>);

// template void PointCloudPerception<ColoredPointT>::ApplyTransformToPointCloud(Eigen::Affine3f,
// 		boost::shared_ptr<pcl::PointCloud<ColoredPointTNormal>>);


// template void PointCloudPerception<PointT>::FusePointCloudPair(
//     const boost::shared_ptr<pcl::PointCloud<PointT>>,
//     const boost::shared_ptr<pcl::PointCloud<PointT>>,
//     boost::shared_ptr<pcl::PointCloud<PointTNormal>>, Eigen::Matrix4f*);

// template void PointCloudPerception<ColoredPointT>::FusePointCloudPair(
//     const boost::shared_ptr<pcl::PointCloud<ColoredPointT>>,
//     const boost::shared_ptr<pcl::PointCloud<ColoredPointT>>,
//     boost::shared_ptr<pcl::PointCloud<ColoredPointTNormal>>, Eigen::Matrix4f*);


// template void PointCloudPerception<PointT>::FuseMultiPointClouds(
//     const std::vector< boost::shared_ptr<pcl::PointCloud<PointT>> >,
//     boost::shared_ptr<pcl::PointCloud<PointTNormal>>);


// template void PointCloudPerception<ColoredPointT>::FuseMultiPointClouds(
//     const std::vector< boost::shared_ptr<pcl::PointCloud<ColoredPointT>> >,
//     boost::shared_ptr<pcl::PointCloud<ColoredPointTNormal>> );


// int main(int argc, char** argv) {
// 	//GeometryAlignmentRepresentation<ColoredPointTNormal> point_representation;
// 	pcl::console::TicToc tt;

// 	PointCloudPerception<ColoredPointT, ColoredPointTNormal> test;
// 	std::string test_file(argv[1]);// = "test_pcd_top.pcd";
// 	//pcl::PointCloud<ColoredPointT>::Ptr cloud(new pcl::PointCloud<ColoredPointT>);
// 	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(new pcl::PointCloud<ColoredPointT>);

// 	test.LoadPCDFile(test_file, cloud);
// 	test.OutlierRemoval(cloud);

// 	//test.DownSample(cloud);
// 	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals(
// 		new pcl::PointCloud<pcl::Normal>);

//   tt.tic ();
// 	test.EstimateNormal(cloud, normals);
// 	std::cout << tt.toc () / 1000.0 << std::endl;

// 	test.VisualizePointCloud(cloud);

// 	tt.tic();
// 	double resolution = 32.0;
//   pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
//   boost::shared_ptr<pcl::PointCloud<PointT>> cloud2(new pcl::PointCloud<PointT>);
//   pcl::copyPointCloud(*cloud, *cloud2);
//   octree.setInputCloud (cloud2);
//   octree.addPointsFromInputCloud ();
//   PointT searchPoint;
//   std::vector<int> pointIdxRadiusSearch;
//   std::vector<float> pointRadiusSquaredDistance;
//   double radius = 100;
//   int tmp_num_round = 10;
//   for (int round = 0; round < tmp_num_round; ++round) {
//   	int tmp_id = rand() % (cloud->size());
//   	searchPoint.x = cloud->points[tmp_id].x;
//   	searchPoint.y = cloud->points[tmp_id].y;
// 		searchPoint.z = cloud->points[tmp_id].z;
//   	octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch,
//   											pointRadiusSquaredDistance);
//   	//std::cout << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << std::endl;
// 		//std::cout << pointIdxRadiusSearch.size() << std::endl;
// 		for (int i = 0; i < pointIdxRadiusSearch.size(); ++i) {
// 			double dot_product = 0;
// 			for (int j = 0; j < 3; ++j){
// 				dot_product += (*normals)[i].normal[j] * (*normals)[tmp_id].normal[j];
// 			}
// 			// std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
//    //              << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
//    //              << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
//    //              << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//       // Just to make sure the index is actually consistent.
// 			// std::cout << "    "  <<   cloud2->points[ pointIdxRadiusSearch[i] ].x
//    //              << " " << cloud2->points[ pointIdxRadiusSearch[i] ].y
//    //              << " " << cloud2->points[ pointIdxRadiusSearch[i] ].z
//    //              << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
// 		}
// 	}
// 	std::cout << "Elapsed time: " << tt.toc () / 1000.0 << std::endl;

// 	test.VisualizePointCloud(cloud);

// 	// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
// 	// Eigen::Vector4d plane_coeffs;
// 	// test.FindPlane(cloud, &plane_coeffs, inliers);
// 	// std::cout << plane_coeffs.transpose() << std::endl;


//   //test.VisualizePointCloudAndNormal(cloud, normals);
// 	std::cout << "Input 6 dof camera" << std::endl;
// 	Eigen::VectorXd camera_pose(6);
// 	for (int i = 0; i < 6; ++i) {
// 		cin >> camera_pose(i);
// 	}

// 	Eigen::Matrix3f M_rot = (Eigen::AngleAxisf(camera_pose(3) / 180.0 * M_PI, Eigen::Vector3f::UnitX())
//     * Eigen::AngleAxisf(camera_pose(4) / 180.0 * M_PI, Eigen::Vector3f::UnitY())
// 		* Eigen::AngleAxisf(camera_pose(5) / 180.0 * M_PI, Eigen::Vector3f::UnitZ())).toRotationMatrix();
// 	Camera camera;
// 	camera.pose(0,3) = camera_pose(0);
// 	camera.pose(1,3) = camera_pose(1);
// 	camera.pose(2,3) = camera_pose(2);
// 	// camera.pose.translation() = camera_pose.head(3);
// 	camera.pose.linear() = M_rot;

// 	camera.fx = 570;
// 	camera.fy = 570;
// 	camera.img_height = 512;
// 	camera.img_width = 512;
// 	test.ApplyTransformToPointCloud(camera.pose.inverse(), cloud);

// 	std::cout << "Input stride size for image completion" << std::endl;
// 	int stride;
// 	std::cin >> stride;
// 	test.ProjectColoredPointCloudToCameraImagePlane(cloud, camera), stride;

// 	return 0;
// }

