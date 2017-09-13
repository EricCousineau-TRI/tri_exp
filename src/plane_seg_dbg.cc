#include <cstdlib>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <ctime>
#include <pcl/console/time.h>
#include <pcl/common/pca.h>

#include <pcl/visualization/pcl_visualizer.h>

// Need to incorporate implementation.
#include <pcl/search/impl/search.hpp>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/octree/octree_search.h>

// Err...
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

using namespace std;

typedef pcl::PointXYZRGB ColoredPointT;
typedef pcl::PointXYZ PointT;

typedef ColoredPointT T;

int main() {
  const string file_name = "src/pre_subtract.pcd";

  boost::shared_ptr<pcl::PointCloud<T>> cloud(new pcl::PointCloud<T>());

  cout << "Reading..." << endl;
  pcl::PCDReader reader;
  reader.read<T>(file_name, *cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  std::cout << "remove Nan size" << indices.size() << std::endl;

  cout << "Before voxel: " << cloud->size() << endl;
  pcl::VoxelGrid<T> grid;
  const double leaf_size = 0.001;
  grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  grid.setInputCloud(cloud);
  grid.filter(*cloud);
  cout << "After voxel: " << cloud->size() << endl;

  pcl::io::savePCDFileASCII("pre_subtract_fixed.pcd", *cloud);

  // int x = system("bash -c pwd");
  // (void)x;

  double dist_threshold = 0.005;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());

  // Does not work well:
  /*
  {
    pcl::SACSegmentation<T> seg;
    // Optional
    // seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_threshold);

    seg.setInputCloud(cloud);
    seg.setMaxIterations(200);
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.segment(*inliers, *coefficients);
  }
  */

  // Works well:
  // from: pcl/tools/sac_segmentation_plane.cpp
  {
    pcl::SampleConsensusModelPlane<T>::Ptr model(
        new pcl::SampleConsensusModelPlane<T>(cloud));
    pcl::RandomSampleConsensus<T> sac (model, dist_threshold);
    sac.setMaxIterations (50);
    bool res = sac.computeModel ();
    
    // vector<int> inliers;
    sac.getInliers (inliers->indices);
    // Eigen::VectorXf coefficients;
    // sac.getModelCoefficients(coefficients);
  }

  int num_before = cloud->size();
  pcl::ExtractIndices<T> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);

  typename pcl::PointCloud<T>::Ptr plane(new pcl::PointCloud<T>());
  typename pcl::PointCloud<T>::Ptr non_plane(new pcl::PointCloud<T>());
  extract.filter(*plane);

  extract.setNegative (true);
  extract.filter(*non_plane);

  cout << "Inlier: " << inliers->indices.size() << endl;
  cout << "Plane: " << plane->size() << endl;
  cout << "Size diff: " << cloud->size() - (plane->size() + non_plane->size()) << endl;
  // Eigen::Map<Eigen::VectorXf> coeffs(coefficients->values.data(),
  //     coefficients->values.size());
  // cout << "Coeffs: " << coeffs.transpose() << endl;

  // Visualize.
  pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
  // auto tf = Eigen::Affine3f::Identity();
  // viewer.addCoordinateSystem(0.2, tf); // Causes segfault.
  pcl::visualization::PointCloudColorHandlerCustom<ColoredPointT> green(plane, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<ColoredPointT> red(non_plane, 255, 0, 0);
  viewer.addPointCloud(plane, green, "Plane");
  viewer.addPointCloud(non_plane, red, "Non-plane");

  // Display the point cloud until 'q' key is pressed.
  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }

  return 0;
}
