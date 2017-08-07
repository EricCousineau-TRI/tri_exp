#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni2_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <boost/thread/thread.hpp>


// This class wraps over the openni grabber class to provide services to access
// the point cloud.
class OpenNiComm{
 public:
  OpenNiComm();
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> GetCurrentPointCloud();
  cv::Mat GetCurrentRGBImage();
  cv::Mat GetCurrentDepthImage();

  void Run() {
    camera_interface_->start ();
  }

  void Stop() {
    camera_interface_->stop();
  }

 private:
 	void rgb_depth_image_cb_(const boost::shared_ptr<pcl::io::Image>& rgb_image,
      const boost::shared_ptr<pcl::io::DepthImage>& depth_image, 
      float reciprocalFocalLength);
 	void cloud_cb_ (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>> & cloud);

  pcl::Grabber* camera_interface_;

  bool request_to_capture_rgb_;
  bool request_to_capture_depth_;
  bool request_to_capture_pointcloud_;
  bool updated_rgb_;
  bool updated_depth_;
  bool updated_pointcloud_;

  // The stored values get deep copied in the callback function whenver 
  // request_to_captured set to be true.
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> stored_cloud_;
  //boost::shared_ptr<pcl::io::Image> stored_image;
  //boost::shared_ptr<pcl::io::DepthImage> stored_depth_image;
  // Type CV_8UC3.
  cv::Mat stored_image_;
  // Type CV_16UC1
  cv::Mat stored_depth_image_;
};