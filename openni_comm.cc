#include "openni_comm.h"
#include <opencv2/highgui/highgui.hpp>

OpenNiComm::OpenNiComm(){
	camera_interface_ = new pcl::io::OpenNI2Grabber();

 	// Initialize the camera interface.
  boost::function<void (const boost::shared_ptr<pcl::io::Image>&,
  		const boost::shared_ptr<pcl::io::DepthImage>&, float)> rgb_depth_fun_ =
      boost::bind (&OpenNiComm::rgb_depth_image_cb_, this, _1, _2, _3);
  camera_interface_->registerCallback(rgb_depth_fun_);

  boost::function<void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>&)> 
  		cloud_fun_ = boost::bind (&OpenNiComm::cloud_cb_, this, _1);
  camera_interface_->registerCallback(cloud_fun_);
  
  request_to_capture_rgb_ = false;
  request_to_capture_pointcloud_ = false;
  request_to_capture_depth_ = false;
  updated_rgb_ = false;
  updated_depth_ = false;
  updated_pointcloud_ = false;

};

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> OpenNiComm::GetCurrentPointCloud() {
	request_to_capture_pointcloud_ = true;
	// Wait until the point cloud is acquired. 
	while (!updated_pointcloud_){
		// Sleep the main thread to wait for device call back function.
		boost::this_thread::sleep (boost::posix_time::seconds (0.001));
	};
	request_to_capture_pointcloud_ = false;
	updated_pointcloud_ = false;
	return stored_cloud_;
}

cv::Mat OpenNiComm::GetCurrentRGBImage() {
	request_to_capture_rgb_ = true;
	while (!updated_rgb_) {
		// Sleep the main thread to wait for device call back function.
		boost::this_thread::sleep (boost::posix_time::seconds (0.001));
	};
	request_to_capture_rgb_ = false;
	updated_rgb_ = false;

	cv::namedWindow("test rgb");
	cv::imshow("cv_image", stored_image_);
	cv::waitKey(0);
	return stored_image_;
}

cv::Mat OpenNiComm::GetCurrentDepthImage() {
	request_to_capture_depth_ = true;
	while (!updated_depth_) {
		//std::cout << "not yet updated" << std::endl;
		boost::this_thread::sleep (boost::posix_time::seconds (0.001));
	};
	request_to_capture_depth_ = false;
	updated_depth_ = false;

	cv::namedWindow("test depth");
	cv::imshow("cv_depth_image", stored_depth_image_);
	cv::waitKey(0);
	return stored_depth_image_;
}

void OpenNiComm::rgb_depth_image_cb_(const boost::shared_ptr<pcl::io::Image>& rgb_image,
     const boost::shared_ptr<pcl::io::DepthImage>& depth_image, 
    float reciprocalFocalLength) {
	//std::cout << "cb_rgb_depth" << std::endl;
	if (request_to_capture_rgb_) {
		stored_image_ = cv::Mat(rgb_image->getHeight(), rgb_image->getWidth(), CV_8UC3);
    rgb_image->fillRGB(stored_image_.cols, stored_image_.rows, stored_image_.data,
        stored_image_.step);
    // Convert from rgb to opencv bgr convention.
   	cv::cvtColor(stored_image_, stored_image_, CV_RGB2BGR);
    updated_rgb_ = true;	
	}
	if (request_to_capture_depth_) {
		// Because getData() returns const and opencv mat does not take const void*.
		void * buffer = new char[depth_image->getDataSize()];
		memcpy(buffer, depth_image->getData(), depth_image->getDataSize()); 
	  stored_depth_image_ = cv::Mat(depth_image->getHeight(), depth_image->getWidth(), 
	  		CV_16UC1, buffer, depth_image->getStep());
	  //depth_image->fillDepthImageRaw(stored_depth_image_.cols, stored_depth_image_.rows, 
	  //	stored_depth_image_.data);
		
		updated_depth_ = true;
	}
}

void OpenNiComm::cloud_cb_(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>> & cloud) {
	 if (request_to_capture_pointcloud_) {
	   stored_cloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>(
	   		new pcl::PointCloud<pcl::PointXYZRGBA>);
	   pcl::copyPointCloud(*cloud, *stored_cloud_);
 	   updated_pointcloud_ = true;
	 }
}


// int main() {
// 	OpenNiComm test;
// 	test.Run();
// 	test.GetCurrentDepthImage();
// 	test.GetCurrentRGBImage();
// 	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = test.GetCurrentPointCloud();
// 	pcl::io::savePCDFileASCII("test_openni_comm.pcd", *cloud);
// 	test.Stop();
// }