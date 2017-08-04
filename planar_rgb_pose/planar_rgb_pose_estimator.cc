#include "planar_rgb_pose_estimator.h"
#include <ctime>

PlanarRGBPoseEstimator::PlanarRGBPoseEstimator(
    cv::Ptr<cv::cuda::DescriptorMatcher> matcher, double nn_match_ratio) {
  nn_match_ratio_ = nn_match_ratio;
  matcher_ = matcher;
}

bool PlanarRGBPoseEstimator::EstimatePlanarPose(const cv::Mat& image, 
    cv::Ptr<cv::cuda::ORB> orb_detector,  PlanarRGBModel& model_template, 
	Eigen::Isometry2d* pose_est) {
	
	cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
	cv::cuda::GpuMat frame_gpu(image_gray);

	cv::cuda::GpuMat features;
	std::vector<cv::KeyPoint> kp;
	orb_detector->detectAndCompute(frame_gpu, cv::noArray(), kp, features);

	// Perform KNN matching.
	std::vector< std::vector<cv::DMatch> > matches;
	std::vector<cv::KeyPoint> matched1, matched2;

	int num_k = 2;
	matcher_->knnMatch(model_template.GetModelFeatures(), features, matches, num_k);
	// Filter out the matches that the second nearest neighbor has a much bigger 
	// distance than the first.
	for(unsigned i = 0; i < matches.size(); i++) {
	    if(matches[i][0].distance < nn_match_ratio_ * matches[i][1].distance) {
	        matched1.push_back(model_template.GetKeyPoints()[matches[i][0].queryIdx]);
	        matched2.push_back(kp[matches[i][0].trainIdx]);
	    }
	}
	if (matched1.size() >= 4) {
		std::vector<cv::Point2f> points_src;
		std::vector<cv::Point2f> points_dst;
    for(unsigned i = 0; i < matched1.size(); i++) {
        points_src.push_back(matched1[i].pt);
        points_dst.push_back(matched2[i].pt);
    }
		cv::Mat affine_t = estimateRigidTransform(points_src, points_dst, false);
		
		// std::cout << affine_t << std::endl;
		// homography = Mat::eye(3, 3, CV_64F);
		Eigen::Isometry2d relative_pose;
		for (int i = 0; i < 2; ++i) { 
			for (int j = 0; j < 3; ++j) {
			    relative_pose(i,j) = affine_t.at<double>(i,j);
			}
		}
		*pose_est = model_template.GetRegisteredPose() * relative_pose;

		return true;
	} else {
		return false;
	}
}

int main(int argc, char** argv) {
	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	Eigen::Isometry2d pose = Eigen::Isometry2d::Identity();	
	PlanarRGBModel test_model(image, pose);
	cv::Ptr<cv::cuda::ORB> orb = cv::cuda::ORB::create();
	test_model.GenerateCudaORBFeatures(orb);
	cv::Ptr< cv::cuda::DescriptorMatcher > matcher = 
		cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
	PlanarRGBPoseEstimator test_estimator(matcher);
	Eigen::Isometry2d test_pose;
	clock_t begin_time, end_time;
	begin_time = clock();
	test_estimator.EstimatePlanarPose(image, orb, test_model, &test_pose);
 	end_time = clock();
  std::cout << "Elapsed time: " << double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl; 

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			std::cout << test_pose(i,j) << " ";
		}
		std::cout << std::endl;
	}
    
}