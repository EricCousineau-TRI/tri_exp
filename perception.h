#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <memory>
#include <sys/time.h>

#include <cmath>

typedef pcl::PointXYZRGBA RGBD;
typedef pcl::PointXYZ POINTONLY;
// This class provides the interface to several PCL algorithms. 
// 1) Statistical removal of outliers. 
// 2) Segmentation of the table.
// 3) Point cloud normal estimation.
// 4) Fuse multiple point clouds.
// 5) Apply transformation to point cloud.
// 6) 

// Todo(Jiaji): consider templating the class so input could be point cloud and
// colored point cloud.
class PointCloudPerception {
  public:
  	PointCloudPerception() {};  	
  	
  	void LoadColoredPCDFile(std::string file_name, 
  		pcl::PointCloud<RGBD>::Ptr cloud);
  	
  	void VisualizePointCloud(pcl::PointCloud<RGBD>::ConstPtr cloud, 
  		Eigen::Affine3f tf = Eigen::Affine3f::Identity());

  	void ApplyTransformToPointCloud(Eigen::Affine3f tf, 
  		pcl::PointCloud<RGBD>::Ptr cloud);

  	void OutlierRemoval(pcl::PointCloud<RGBD>::Ptr cloud, 
  		int num_neighbor = 50, double std_dev_threshold = 1.0);
  	
  	// Get the plane coeffcients. ax + by + cz + d = 0, returned in vector4d.
  	void SegmentPlane(const pcl::PointCloud<RGBD>::ConstPtr cloud, 
  		Eigen::Vector4d* coeffs_plane,  pcl::PointIndices::Ptr inliers, 
  		double dist_threshold = 0.01);

  	// Points normals estimation.

  	// Fuse multiple point clouds.

  private:

};