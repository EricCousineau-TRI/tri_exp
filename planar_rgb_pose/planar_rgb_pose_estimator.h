#include "planar_rgb_model.h"
#include <memory>
// This class loads a rgb template and a feature detector to do template matching.
// KNN matching is used. Then a rigid transformation is estimated.
// Todo (Jiaji): right now only takes orb feature. Use others later.
// Provides interface to estimate pose given a query image.  
class PlanarRGBPoseEstimator{
 public:	
  PlanarRGBPoseEstimator(cv::Ptr<cv::cuda::DescriptorMatcher> matcher, 
  						 double nn_match_ratio = 0.8);
  bool EstimatePlanarPose(const cv::Mat& image, cv::Ptr<cv::cuda::ORB> orb_detector, 
      PlanarRGBModel& model_template, Eigen::Isometry2d* pose_est);
 private:
  double nn_match_ratio_;
  cv::Ptr<cv::cuda::DescriptorMatcher> matcher_;
};