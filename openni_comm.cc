#include "openni_comm.h"
#include <opencv2/highgui/highgui.hpp>
OpenNiComm::OpenNiComm() {
	camera_interface_ = new pcl::io::OpenNI2Grabber();
  stored_cloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(
      new pcl::PointCloud<pcl::PointXYZRGB>);

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

  camera_interface_->start ();

};

void OpenNiComm::GetCurrentPointCloud(
 		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> output_cloud) {
	std::unique_lock<std::mutex> lock(mtx_);
	request_to_capture_pointcloud_ = true;
	// Wait until the point cloud is acquired.
	// The updated_pointcloud boolean variable also needs to be locked 
	// properly.
	while (!updated_pointcloud_){
		// Sleep the main thread to wait for device call back function.
		lock.unlock();
		usleep(0.01 * 1e+6);
		lock.lock();
	};
	request_to_capture_pointcloud_ = false;
	updated_pointcloud_ = false;
 	//output_cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(
 	//	new pcl::PointCloud<pcl::PointXYZRGB>);
 	//mtx_.lock();
	pcl::copyPointCloud(*stored_cloud_, *output_cloud);
	//mtx_.unlock();
	std::cout << "Get cloud size: " << output_cloud->size();
	//return stored_cloud_;
}

cv::Mat OpenNiComm::GetCurrentRGBImage() {
	std::unique_lock<std::mutex> lock(mtx_);
	request_to_capture_rgb_ = true;
	while (!updated_rgb_) {
		// Sleep the main thread to wait for device call back function.
		lock.unlock();
		usleep(0.01 * 1e+6);
		lock.lock();
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
	std::unique_lock<std::mutex> lock(mtx_);
	while (!updated_depth_) {
		lock.unlock();
		usleep(0.01 * 1e+6);
		lock.lock();
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
	std::unique_lock<std::mutex> lock(mtx_);
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
	std::lock_guard<std::mutex> lock(mtx_);
	 if (request_to_capture_pointcloud_) {
	   //mtx_.lock();	
 	   stored_cloud_->resize(cloud->size());
 	   std::cout << "cloud cb" << std::endl;
 	   for (int i = 0; i < cloud->size(); ++i) {
 	     stored_cloud_->points[i].x = cloud->points[i].x;
 	     stored_cloud_->points[i].y = cloud->points[i].y;
 	     stored_cloud_->points[i].z = cloud->points[i].z;
 	     stored_cloud_->points[i].r = cloud->points[i].r;
 	     stored_cloud_->points[i].g = cloud->points[i].g;
 	     stored_cloud_->points[i].b = cloud->points[i].b;
 	   }
 	   updated_pointcloud_ = true;
 	   //mtx_.unlock();
     std::cout << "cloud cb completead" << std::endl;
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
