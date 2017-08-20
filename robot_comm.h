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
	RobotComm(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& tool_frame)
		: robot_controller_(tree, tool_frame), robot_state_(&tree, &tool_frame) {
			robot_controller_.Start();
		};

	// "SetTool". Set tool transform with respect to link 7. This could be the 
	// gripper frame or the camera frame.
	void SetToolTransform(Eigen::Isometry3d tf);

	// "MoveQ". Linear movement in joint space.
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
		Eigen::VectorXd q_radian = q / 180.0 * M_PI;
		duration = std::max(0.5, duration);
		robot_controller_.MoveJ(q_radian, duration);
		WaitUntilControlAckDone();
	}

	// "GetQ". Get joint position and print on terminal.
	Eigen::VectorXd GetJointPosition() {
		robot_controller_.GetIiwaState(&robot_state_);
		Eigen::VectorXd q = robot_state_.get_q();
		q = (180 / M_PI) * q;
		std::cout << "Robot Joint: " << q.transpose() << std::endl;
		return q;
	}

	// "MoveC". Linear movement in Cartesian space.
	void MoveToCartesianPosition(const Eigen::Isometry3d& tgt_pose_ee,
														  double duration, double fz = 0, double mu = 0) {
		robot_controller_.GetIiwaState(&robot_state_);
		Eigen::Isometry3d cur_pose_ee = robot_state_.get_X_WT();
  	drake::manipulation::PiecewiseCartesianTrajectory<double> traj =
    	drake::manipulation::PiecewiseCartesianTrajectory<double>::
      	  MakeCubicLinearWithEndLinearVelocity({0.2, duration}, {cur_pose_ee, tgt_pose_ee},
        	                                     drake::Vector3<double>::Zero(),
          	                                   drake::Vector3<double>::Zero());

		robot_controller_.MoveToolFollowTraj(traj, fz, 0, mu);
		WaitUntilControlAckDone();

	}

	// "GetC". Get cartesian pose and print on terminal.
	Eigen::Isometry3d GetCartesianPosition() {
		robot_controller_.GetIiwaState(&robot_state_);
		Eigen::Isometry3d pose_tool = robot_state_.get_X_WT();
		std::cout << "Robot Tool: " << std::endl;
		std::cout << pose_tool.matrix() << std::endl;
		std::cout << "Euler Angles: " << std::endl;
		Eigen::Vector3d rpy = (180.0 / M_PI) * pose_tool.linear().eulerAngles(0, 1, 2); 
		std::cout << rpy.transpose() << std::endl;
		std::cout << "Translation: " << std::endl;
		std::cout << pose_tool.translation().transpose() << std::endl;
		return pose_tool;
	}

	// "GuardedMove". Move in a certain direction until a 
	// certain amount of force is sensed.
	void MoveUntilTouch();

	void SetGripperPositionAndForce(double position, double force) {
		robot_controller_.SetGripperPositionAndForce(position, force);
	}

	// "Close". Close the gripper.
	void CloseGripper() {
		robot_controller_.CloseGripperAndSleep();
	}

	// "Open". Open the gripper.
	void OpenGripper() {
		robot_controller_.OpenGripperAndSleep();
	}

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

 private:
 	drake::jjz::JjzController robot_controller_;

 	drake::jjz::IiwaState robot_state_;
};