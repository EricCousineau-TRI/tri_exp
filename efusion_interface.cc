#include "efusion_interface.h"
#include <time.h>
#include <stdlib.h>

// EFusionInterface::EFusionInterface() {
//  	Resolution::getInstance(640, 480);
//   //Intrinsics::getInstance(528, 528, 320, 240);
//   Intrinsics::getInstance(550, 550, 297.31, 228.32);
// 	pangolin::Params windowParams;
//   windowParams.Set("SAMPLE_BUFFERS", 0);
//   windowParams.Set("SAMPLES", 0);
//   pangolin::CreateWindowAndBind("Main", 1280, 980, windowParams);

//  //  double icpErrThresh = 5e-05;
//  //  double covThresh = 1e-05;
//  //  double icpCountThresh = 40000;
//  //  //double timeDelta = std::numeric_limits<int>::max() / 2;
//  //  double timeDelta = 200;
//  //  bool flag_openloop = 0;
// 	// efusion_ = std::make_unique<ElasticFusion>(timeDelta,
// 	// 																					icpCountThresh,
//  //                                        		icpErrThresh,
//  //                                        		covThresh,
//  //                                        		!flag_openloop); 
//   efusion_ = std::make_unique<ElasticFusion>();
// 	efusion_->setConfidenceThreshold(2.5);
// 	camera_interface_ = std::make_unique<OpenNiComm>();
// 	// Sleep sometime for camera to initialize connection.
// 	usleep(0.5 * 1e+6);
// }

void EFusionInterface::ProcFrame(const Eigen::Isometry3d camera_pose) {
	
	cv::Mat rgb_img = camera_interface_->GetCurrentRGBImage();
	cv::cvtColor(rgb_img, rgb_img, CV_BGR2RGB);
	cv::Mat depth_img = camera_interface_->GetCurrentDepthImage();
	
	Eigen::Matrix4f camera_pose_prior;
	camera_pose_prior.matrix() = camera_pose.matrix().cast<float>();
	bool flag_prior = false;
	//time_t t = time(NULL);	
	struct timeval tp;
  gettimeofday(&tp, NULL);
  long long t = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000;
	//std::cout << "timestamp: " << t << std::endl;
	//std::cout << camera_pose_prior.matrix() << std::endl;
	
	efusion_->processFrame(rgb_img.data, reinterpret_cast<unsigned short*>(depth_img.data), 
		t, &camera_pose_prior, 1.0, flag_prior);
}


void EFusionInterface::GetFusedPointCloud(
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> cloud) {
 		
  double confidence_threshold = efusion_->getConfidenceThreshold();
  GlobalModel& globalModel = efusion_->getGlobalModel();
  Eigen::Vector4f * mapData = globalModel.downloadMap();

  int validCount = 0;
  for (unsigned int i = 0; i < globalModel.lastCount(); i++) {
    Eigen::Vector4f pos = mapData[(i * 3) + 0];
    if(pos[3] > confidence_threshold){
        validCount++;
    }
  }
  std::cout << "Total count: " << globalModel.lastCount() << std::endl;
  std::cout << "Number of points over confidence threshold: " << validCount << std::endl;
	cloud->reserve(validCount);  

  for (unsigned int i = 0; i < globalModel.lastCount(); i++) {
  	pcl::PointXYZRGBNormal pt;
    Eigen::Vector4f pos = mapData[(i * 3) + 0];
   //  if (i%1000 == 1) {
   //  	std::cout << pos[3] <<"," << confidence_threshold << std::endl;
  	// }
    if(pos[3] > confidence_threshold) {
    	pt.x = pos[0];
    	pt.y = pos[1];
    	pt.z = pos[2];

      Eigen::Vector4f col = mapData[(i * 3) + 1];
      pt.r = int(col[0]) >> 16 & 0xFF;
      pt.g = int(col[0]) >> 8 & 0xFF;
      pt.b = int(col[0]) & 0xFF;
      Eigen::Vector4f nor = mapData[(i * 3) + 2];

      nor[0] *= -1;
      nor[1] *= -1;
      nor[2] *= -1;
      pt.normal_x = nor[0];
      pt.normal_y = nor[1];
      pt.normal_z = nor[2];
      cloud->push_back(pt);
    }
  }
  // Close file
  //fs.close ();

  delete [] mapData;	
}




