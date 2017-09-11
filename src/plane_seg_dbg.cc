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

using namespace std;

typedef pcl::PointXYZRGB ColoredPointT;
typedef pcl::PointXYZ PointT;

typedef ColoredPointT T;

int main() {
  const string file_name = "pre_subtract.pcd";

  boost::shared_ptr<pcl::PointCloud<T>> cloud(new pcl::PointCloud<T>());

  pcl::PCDReader reader;
  reader.read<T>(file_name, *cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
  std::cout << "remove Nan size" << indices.size() << std::endl;

  // Visualize.
  pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
  auto tf = Eigen::Affine3f::Identity();
  viewer.addCoordinateSystem(0.2, tf);
  pcl::visualization::PointCloudColorHandlerCustom<ColoredPointT> single_color (cloud, 0, 255, 0);
  // viewer.addPointCloud(cloud, rgb, "Cloud");
  viewer.addPointCloud(cloud, single_color, "Cloud");
  std::cout << "to display" << std::endl;
  // Display the point cloud until 'q' key is pressed.
  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }

  return 0;
}
