#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni2_grabber.h>

#include <boost/thread/thread.hpp>

class HandEye {
 public:
   // Set the transformation of the camera with respect to hand/tool frame.
   void SetEyeTransformationWrtToolFrame(Eigen::Isometry3d tf_eye_wrt_hand);

	// Get the pose of the camera with respect to robot base. 
	Eigen::Isometry3d GetCameraPoseWrtRobotBase(
		Eigen::Isometry3d tf_tool_wrt_base);

	// Grab current point cloud from the openni camera (xtion pro) and save a 
	// copy for the class.	
	void GrabCurrentPointCloud(); 
 private:
	bool flag_acquire_new_data;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pt_cloud;

};