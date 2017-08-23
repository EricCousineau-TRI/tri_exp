#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/examples/kuka_iiwa_arm/jjz_controller.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/manipulation/util/trajectory_utils.h"
class RobotComm {
 public:
	RobotComm(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& tool_frame);

	// "SetTool". Set tool transform with respect to link 7. This could be the 
	// gripper frame or the camera frame.
	void SetToolTransform(Eigen::Isometry3d tf){};

	// "MoveQ". Linear movement in joint space.
	void MoveToJointPositionDegrees(const Eigen::VectorXd & q, double duration,
			bool flag_blocking = true);

	void MoveToJointPositionRadians(const Eigen::VectorXd & q, double duration, 
			bool flag_blocking = true);

	// "GetQ". Get joint position and print on terminal.
	Eigen::VectorXd GetJointPosition();

	// "MoveC". Linear movement in Cartesian space.
	void MoveToCartesianPose(const Eigen::Isometry3d& tgt_pose_ee,
													 double duration, double fz = 0, double mu = 0);

	// "GetC". Get cartesian pose and print on terminal.
	Eigen::Isometry3d GetCartesianPose();

	// "GuardedMove". Move in a certain direction until a 
	// certain amount of force is sensed.
	void MoveUntilTouch();

	void SetGripperPositionAndForce(double position, double force);

	// "Close". Close the gripper.
	void CloseGripper();
	// "Open". Open the gripper.
	void OpenGripper();

	void WaitUntilControlAckDone();

 private:
 	drake::jjz::JjzController robot_controller_;

 	drake::jjz::IiwaState robot_state_;
};