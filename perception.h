#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <memory>
#include <sys/time.h>

#include <cmath>
// This class provides the interface to several PCL algorithms. 
// 1) Segmentation of the table.
// 2) Fuse multiple point clouds
// 3) Point cloud normal estimation.
class Perception {
  public:
  	Perception();
  	// Set the transformation of the camera with respect to hand/tool frame.
    void SetEyeTransformationWrtToolFrame(Eigen::Isometry3d tf_eye_wrt_hand);
    
    // Get the pose of the camera with respect to robot base. 
    Eigen::Isometry3d GetCameraPoseWrtRobotBase(
    	Eigen::Isometry3d tf_tool_wrt_base);
    
    // Grab current point cloud from the openni camera (xtion pro) and save a 
    // copy for the class.	
  	void GrabCurrentPointCloud(); 
  	
  	// Get the plane coeffcients. ax + by + cz + d = 0, returned in vector4d.
  	Eigen::Vector4d SegmentTable();


  private:
  	bool flag_acquire_new_data;
  	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pt_cloud;

};