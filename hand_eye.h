#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni2_grabber.h>

#include <boost/thread/thread.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
//#include "drake/lcmt_iiwa_command.hpp"
//#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/examples/kuka_iiwa_arm/jjz_controller.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/manipulation/util/trajectory_utils.h"

#include "openni_comm.h"
#include "perception.h"

// This class provides basic kinematics to move around camera via interfacing with
// robot trajectory controller.
class HandEye {
 public:
  HandEye(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& tool_frame);

  // Set the transformation of the camera with respect to hand/tool frame.
  void SetEyeTransformationWrtHandFrame(const Eigen::Isometry3d& tf_eye_wrt_hand);

	// Get the pose of the camera with respect to robot base.
	Eigen::Affine3f GetCameraPoseWrtRobotBase();

	// Move the camera to a target pose in cartesian straigt-line fashion.
	void MoveCameraToCartesianPose(const Eigen::Isometry3d& tgt_pose, 
			double duration = 2.0);
	void MoveHandToCartesianPose(const Eigen::Isometry3d& tgt_pose, 
			double duration = 2.0);

 	void MoveRobotJointStraightLine(const Eigen::VectorXd& tgt_joint, 
 			double duration = 5.0);

	void Scan(OpenNiComm& camera_interface, 
			 PointCloudPerception & perception_proc, 
			const std::vector<Eigen::VectorXd>& joint_angles, 
			double duration_per_move, 
			boost::shared_ptr<pcl::PointCloud<ColoredPointTNormal>> fused_cloud);	

	// Grab current point cloud from the openni camera (xtion pro) and save a
	// copy for the class.
	//void GrabCurrentPointCloud();
	//void SaveCurrentPointCloud(std::string file_name);
 private:
 	void UpdateRobotState();
 	void MoveEECartesianStraightLine(const Eigen::Isometry3d& tgt_pose_ee, 
 			double duration = 2.0);
 	void WaitUntilControlAckDone();

 	drake::jjz::JjzController robot_controller_;
 	//OpenNiComm camera_interface_;
 	drake::jjz::IiwaState robot_state_;
 	// Camera frame with respect to hand frame.
 	Eigen::Isometry3d tf_eye_wrt_hand_;
 	//const RigidBodyTree<double>& robot_;
  //const RigidBody<double>& end_effector_;

	bool flag_acquire_new_data;

	//PointCloudPerception pcl_interface;

};
