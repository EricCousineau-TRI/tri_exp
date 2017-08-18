#include "openni_comm.h"
#include <iostream>
#include <memory>
#include <fstream>

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/examples/kuka_iiwa_arm/jjz_controller.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/manipulation/util/trajectory_utils.h"

// PCL (openni grabber) and april tag related stuffs.
#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sys/time.h>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
#include <AprilTags/TagFamily.h>


const std::string kPath =
    "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf";
const std::string kEEName = "iiwa_link_7";
const Eigen::Isometry3d kBaseOffset = Eigen::Isometry3d::Identity();

constexpr double kTagSize =  0.173;

class AprilTagPerception {
 public:
  // Default is 36h11 family with A4 paper print size.
  AprilTagPerception() : tag_code_(AprilTags::tagCodes36h11) {
    tag_size_ = kTagSize;
    fx_ = 515.5;
    fy_ = 515.5;
    px_ = 304.5;
    py_ = 227.2;
    // Initialize the april tag detector.
    tag_detector_ = new AprilTags::TagDetector(tag_code_);
  }

  void SetAprilTagCodeFamily(AprilTags::TagCodes tag_code) {
    tag_code_ = tag_code;
    // Reinitialize the tag detector.
    tag_detector_ = new AprilTags::TagDetector(tag_code_);
  }

  void SetAprilTagCodeSize(double tag_size) {
    tag_size_ = tag_size;
  }

  bool GetDetection(const cv::Mat cv_image, Eigen::Isometry3d* april_tag_pose) {
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;   
   	cv::Mat image_gray;
    cv::cvtColor(cv_image, image_gray, CV_BGR2GRAY);
    vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(
        image_gray);
    bool is_detected = (detections.size() == 1);
    if (is_detected) {
    	*april_tag_pose = detections[0].getRelativeTransform(tag_size_, fx_, fy_, 
  			px_, py_);
  	}
  	// std::cout << april_tag_pose.linear() << std::endl;
    // std::cout << april_tag_pose.translation() << std::endl;
    return is_detected;
    
  }
 private:
  // April Detector.
  AprilTags::TagDetector* tag_detector_;
  // Property of the april tag.
  AprilTags::TagCodes tag_code_;
  double tag_size_;
  // Instrinsics of the rgb camera. Focal Length and optical center.
  double fx_;
  double fy_;
  double px_;
  double py_;
};

class MinimumRobot{
 public:
	MinimumRobot(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& tool_frame)
		: robot_controller_(tree, tool_frame), robot_state_(&tree, &tool_frame) {
			robot_controller_.Start();
	}

	void MoveToJointPosition(const Eigen::VectorXd & q, double duration) {
		if ( q.size() != 7 ) {
			std::cout << "Not 7 number" << std::endl;
			return;
		}
		for (int i = 0; i < q.size(); ++i) {
			if (q[i] > 180 || q[i] < -180) {
				std::cout << "out of limit " << std::endl;
				return;
			}
		}
		duration = std::max(0.5, duration);
		robot_controller_.MoveJ(q, duration);
		WaitUntilControlAckDone();
	}

	Eigen::Isometry3d GetCartesianPosition() {
		robot_controller_.GetIiwaState(&robot_state_);
		Eigen::Isometry3d pose_tool = robot_state_.get_X_WT();
		// std::cout << "Robot Tool: " << std::endl;
		// std::cout << pose_tool.matrix() << std::endl;
		// std::cout << "Euler Angles: " << std::endl;
		// Eigen::Vector3d rpy = (180.0 / M_PI) * pose_tool.linear().eulerAngles(0, 1, 2); 
		// std::cout << rpy.transpose() << std::endl;
		// std::cout << "Translation: " << std::endl;
		// std::cout << pose_tool.translation().transpose() << std::endl;
		return pose_tool;
	}
 private:
	void WaitUntilControlAckDone() {
	 	drake::jjz::PrimitiveOutput output;
		while(true) {
	    // Controller's output, contains status flag.
	    robot_controller_.GetPrimitiveOutput(&output);
	    if (output.status == drake::jjz::PrimitiveOutput::DONE)
	    	break;
		}
		return;
	}
 	drake::jjz::JjzController robot_controller_;

 	drake::jjz::IiwaState robot_state_;
};

int main(int argc, char** argv) {
	AprilTagPerception april_percep;
	OpenNiComm camera_interface;

	RigidBodyTree<double> tree;
	drake::parsers::urdf::AddModelInstanceFromUrdfFile(
		kPath, drake::multibody::joints::kFixed, nullptr, &tree);
	// Use rough caliration from last time to locate the camera frame.
	Eigen::Isometry3d tf_camera_wrt_ee;
  // tf_camera_wrt_ee.matrix() <<
  // 	0.37451879, -0.92700796,  0.        , -0.06087369,
		// 0.92628617,  0.37501423,  0.        ,  0.03164403,
		// 0.04158859, -0.00453463,  1.        ,  0.105     ,
  //   0.        ,  0.        ,  0.        ,  1.0;
  tf_camera_wrt_ee.matrix() <<
        0.4203,   -0.9074,    0.0061,   -0.0468,
    0.9071,    0.4200,   -0.0292,    0.0053,
    0.0239,    0.0178,    0.9996,    0.1455,
         0,         0,         0,    1.0000;
  //Eigen::Isometry3d tf_camera_wrt_hand = drake::jjz::X_ET.inverse() * tf_camera_wrt_ee;
  RigidBodyFrame<double> camera_frame("camera", tree.FindBody(drake::jjz::kEEName),
                                      Eigen::Isometry3d::Identity());
                                      //tf_camera_wrt_ee);

  MinimumRobot robot(tree, camera_frame);

  // Generate calibration grid.
  double gaze_dist = atof(argv[1]);
 	double y_span = atof(argv[2]);
 	double x_span = atof(argv[3]);
 	int num_grid_per_dim = int(atof(argv[4]));

 	Eigen::VectorXd q0 = Eigen::VectorXd::Zero(7);
  q0[1] = -11;
  q0[3] = -69;
  q0[5] = 109;
  q0 = q0 / 180. * M_PI;
  Eigen::Vector3d target_location(0.60, 0, 0);
		std::vector<Eigen::VectorXd> joint_targets =
    	drake::jjz::ComputeCalibrationConfigurations(
  	  		tree, camera_frame, q0, target_location, gaze_dist, 
  	  		y_span, x_span, num_grid_per_dim, num_grid_per_dim);

  double movement_duration = 2.0;
  ofstream output_ss;
  output_ss.open("output.txt");
  for (int i = 0; i < joint_targets.size(); ++i) {
  	robot.MoveToJointPosition(joint_targets[i], movement_duration);
  	cv::Mat img = camera_interface.GetCurrentRGBImage();
  	Eigen::Isometry3d april_pose; 
  	bool is_detected = april_percep.GetDetection(img, &april_pose);
  	Eigen::Isometry3d robot_pose = robot.GetCartesianPosition();
  	if (is_detected) {
    	output_ss << april_pose.matrix() << std::endl;
    	output_ss << robot_pose.matrix() << std::endl;
  }
  }
  output_ss.close();
  while(true) {};
  return 0;
}
