#include "hand_eye.h"

HandEye::HandEye(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& tool_frame)
  : robot_controller_(tree, tool_frame),
  	robot_state_(&tree, &tool_frame) {
  //robot_controller_ = std::make_unique<drake::jjz::JjzController>(tree, tool_frame);
  robot_controller_.Start();
}

void HandEye::SetEyeTransformationWrtHandFrame(
		Eigen::Isometry3d tf_eye_wrt_hand) {
	tf_eye_wrt_hand_ = tf_eye_wrt_hand;
}

// Get the pose of the camera with respect to robot base.
Eigen::Isometry3d HandEye::GetCameraPoseWrtRobotBase() {
	UpdateRobotState();
	Eigen::Isometry3d pose_ee = robot_state_.get_X_WT();
	Eigen::Isometry3d pose_camera = pose_ee * tf_eye_wrt_hand_;
}

void HandEye::MoveCameraToCartesianPose(Eigen::Isometry3d tgt_pose_camera,
																				double duration) {
	Eigen::Isometry3d tgt_pose_ee = tgt_pose_camera * tf_eye_wrt_hand_.inverse();
	MoveEECartesianStraightLine(tgt_pose_ee, duration);
}

void HandEye::MoveHandToCartesianPose(Eigen::Isometry3d tgt_pose_ee,
																			double duration) {
	MoveEECartesianStraightLine(tgt_pose_ee, duration);
}


void HandEye::UpdateRobotState() {
	// The robot controller gets the current robot state from LCM message
	// and update the state. Kinematics cache is updated.
	robot_controller_.GetState(&robot_state_);
}

void HandEye::MoveEECartesianStraightLine(Eigen::Isometry3d tgt_pose_ee,
																				  double duration) {
	UpdateRobotState();
	Eigen::Isometry3d cur_pose_ee = robot_state_.get_X_WT();
  drake::manipulation::PiecewiseCartesianTrajectory<double> traj =
    drake::manipulation::PiecewiseCartesianTrajectory<double>::
        MakeCubicLinearWithEndLinearVelocity({0.0, duration}, {cur_pose_ee, tgt_pose_ee},
                                             drake::Vector3<double>::Zero(),
                                             drake::Vector3<double>::Zero());

  // Todo(Jiaji) : This is hack just to use the existing api.
	double dummy_fz = 0;
	double dummy_mu = 0;
	robot_controller_.MoveToolFollowTraj(traj, dummy_fz, dummy_mu, dummy_mu);

}

int main() {
	const std::string kPath =
      "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf";

  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);

  RigidBodyFrame<double> tool_frame("tool", tree.FindBody(drake::jjz::kEEName),
  		drake::jjz::X_ET);

  //drake::jjz::JjzController controller(tree, tool_frame);
  //OpenNiComm camera_comm;
  HandEye hand_eye_system(tree, tool_frame);
  return 0;
}
