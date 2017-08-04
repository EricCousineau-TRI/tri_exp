#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <vector>
#include <Eigen/Geometry>

// This class implements the template model assuming objects resting on a plane.
// User supplies a image and the object pose with respect to the camera.
// A feature extractor is applied to the image and will store the point-based
// descriptions for template matching.
// Right now, we are using cuda ORB features for fast speed (>=40Hz).
// Todo (Jiaji): use abstract virtual class for other features later. 
class PlanarRGBModel{
 public:
 	PlanarRGBModel(cv::Mat image, Eigen::Isometry2d pose);
	void GenerateCudaORBFeatures(cv::Ptr<cv::cuda::ORB> detector);
	// Optional. If not set, then the feature detector will apply to the entire
	// image.
	void SetPolygonalMask(std::vector<cv::Point2f> mask_pts);
	const cv::cuda::GpuMat & GetModelFeatures() {return features_;};
	const std::vector<cv::KeyPoint> & GetKeyPoints() {return key_points_;}
	Eigen::Isometry2d GetRegisteredPose() {return registered_pose_;}
 private:
 	cv::Mat template_image_;
 	// Registered planar pose.
 	Eigen::Isometry2d registered_pose_; 
 	cv::cuda::GpuMat mask_gpu_;
 	cv::cuda::GpuMat template_image_gpu_;
 	cv::cuda::GpuMat features_;
 	std::vector<cv::KeyPoint> key_points_;
};