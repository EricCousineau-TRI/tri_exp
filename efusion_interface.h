#include "openni_comm.h"
#include <vector>
#include <memory>
#include <ElasticFusion.h>

class EFusionInterface{
 public:	
	EFusionInterface();
	// Use the camera pose to bootstrap elastic fusion. 
 	void ProcFrame(const Eigen::Isometry3d camera_pose);
 	void GetFusedPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> cloud);
 private:
 	std::unique_ptr<ElasticFusion> efusion_;
 	std::unique_ptr<OpenNiComm> camera_interface_;
};