#include "planar_rgb_model.h"

PlanarRGBModel::PlanarRGBModel(cv::Mat image, Eigen::Isometry2d pose) {
  template_image_ = image;
  registered_pose_ = pose;	
  
  cv::Mat template_image_gray;
  cv::cvtColor(template_image_, template_image_gray, CV_BGR2GRAY);
  template_image_gpu_ = cv::cuda::GpuMat(template_image_gray);
  
  cv::Mat matMask = cv::Mat::zeros(template_image_.size(), CV_8UC1);
  cv::cuda::GpuMat mask_gpu_(matMask);

}

void PlanarRGBModel::GenerateCudaORBFeatures(cv::Ptr<cv::cuda::ORB> feature_detector) {
  feature_detector->detectAndCompute(template_image_gpu_, mask_gpu_, 
  									key_points_, features_);
}

void PlanarRGBModel::SetPolygonalMask(std::vector<cv::Point2f> mask_pts) {
  cv::Point *ptMask = new cv::Point[mask_pts.size()];
  const cv::Point* ptContain = { &ptMask[0] };
  int iSize = static_cast<int>(mask_pts.size());
  for (size_t i=0; i<mask_pts.size(); i++) {
    ptMask[i].x = static_cast<int>(mask_pts[i].x);
    ptMask[i].y = static_cast<int>(mask_pts[i].y);
  }
  cv::Mat matMask = cv::Mat::zeros(template_image_.size(), CV_8UC1);
  cv::fillPoly(matMask, &ptContain, &iSize, 1, cv::Scalar::all(255));
  cv::cuda::GpuMat mask_gpu_(matMask);
  delete[] ptMask;
}

// int main(int argc, char** argv) {
// 	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
// 	Eigen::Isometry2d pose = Eigen::Isometry2d::Identity();	
// 	PlanarRGBModel test(image, pose);
// 	cv::Ptr<cv::cuda::ORB> orb = cv::cuda::ORB::create();
// 	test.GenerateCudaORBFeatures(orb);
// }
