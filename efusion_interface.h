#include "openni_comm.h"
#include <vector>
#include <memory>
#include <ElasticFusion.h>

class EFusionInterface{
 public:	
	EFusionInterface() {
	 	Resolution::getInstance(640, 480);
	  //Intrinsics::getInstance(528, 528, 320, 240);
	  Intrinsics::getInstance(550, 550, 297.31, 228.32);
		pangolin::Params windowParams;
	  windowParams.Set("SAMPLE_BUFFERS", 0);
	  windowParams.Set("SAMPLES", 0);
	  pangolin::CreateWindowAndBind("Main", 1240, 980, windowParams);

	 //  double icpErrThresh = 5e-05;
	 //  double covThresh = 1e-05;
	 //  double icpCountThresh = 40000;
	 //  //double timeDelta = std::numeric_limits<int>::max() / 2;
	 //  double timeDelta = 200;
	 //  bool flag_openloop = 0;
		// efusion_ = std::make_unique<ElasticFusion>(timeDelta,
		// 																					icpCountThresh,
	 //                                        		icpErrThresh,
	 //                                        		covThresh,
	 //                                        		!flag_openloop); 
	  efusion_ = std::make_unique<ElasticFusion>();
		efusion_->setConfidenceThreshold(2.5);
		camera_interface_ = std::make_unique<OpenNiComm>();
		// Sleep sometime for camera to initialize connection.
		usleep(0.5 * 1e+6);
	}
	// Use the camera pose to bootstrap elastic fusion. 
 	void ProcFrame(const Eigen::Isometry3d camera_pose);
 	void GetFusedPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> cloud);
 private:
 	std::unique_ptr<ElasticFusion> efusion_;
 	std::unique_ptr<OpenNiComm> camera_interface_;
};