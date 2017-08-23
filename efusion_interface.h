#include "openni_comm.h"
#include <vector>
#include <memory>
#include <ElasticFusion.h>

class EFusionInterface{
 public:	
	EFusionInterface();
	// Use the camera pose to boostrap elastic fusion. 
 	void ProcFrame(const Eigen::Isometry3d camera_pose);
 	void GetFusedPointCloud();
 private:
 	std::unique_ptr<ElasticFusion> efusion_;
 	OpenNiComm camera_interface_;
};