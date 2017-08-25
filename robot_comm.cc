#include "robot_comm.h"
RobotComm::RobotComm(const RigidBodyTree<double>& tree, 
		const RigidBodyFrame<double>& tool_frame)
		: robot_controller_(tree, tool_frame), robot_state_(&tree, &tool_frame) {
			robot_controller_.Start();
			// Need to sleep until getting the first status message back.
			usleep(1e+6 * 0.5);
}

// "MoveQ". Linear movement in joint space.
void RobotComm::MoveToJointPositionDegrees(const Eigen::VectorXd & q, double duration, 
		bool flag_blocking) {
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
	if (flag_blocking) {
		WaitUntilControlAckDone();
	}
}

void RobotComm::MoveToJointPositionRadians(const Eigen::VectorXd & q, double duration, 
		bool flag_blocking) {
	if ( q.size() != 7 ) {
		std::cout << "Not 7 number" << std::endl;
		return;
	}
	for (int i = 0; i < q.size(); ++i) {
		if (q[i] > M_PI || q[i] < - M_PI) {
			std::cout << "out of limit " << std::endl;
			return;
		}
	}
	duration = std::max(0.5, duration);
	robot_controller_.MoveJ(q, duration);
	if (flag_blocking) {
		WaitUntilControlAckDone();
	}
}

// "GetQ". Get joint position and print on terminal.
Eigen::VectorXd RobotComm::GetJointPosition() {
	robot_controller_.GetIiwaState(&robot_state_);
	Eigen::VectorXd q = robot_state_.get_q();
	q = (180 / M_PI) * q;
	std::cout << "Robot Joint: " << q.transpose() << std::endl;
	return q;
}

// "MoveC". Linear movement in Cartesian space.
void RobotComm::MoveToCartesianPose(const Eigen::Isometry3d& tgt_pose_ee,
		double duration, double fz, double mu) {
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
Eigen::Isometry3d RobotComm::GetCartesianPose() {
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

// "GuardedMove". Move in a certain direction until a 
// certain amount of force is sensed.
void RobotComm::MoveUntilTouch(){};


// "Close". Close the gripper.
void RobotComm::CloseGripper() {
	//robot_controller_.CloseGripperAndSleep(0.5);
	robot_controller_.SetGripperPositionAndForce(0, 80);
	usleep(0.5 * 1e+6);
}

// "Open". Open the gripper.
void RobotComm::OpenGripper() {
	robot_controller_.OpenGripperAndSleep(0.25);
}

void RobotComm::SetGripperPositionAndForce(double position, double force) {
	robot_controller_.SetGripperPositionAndForce(position, force);
}


void RobotComm::WaitUntilControlAckDone() {
 	drake::jjz::PrimitiveOutput output;
	while(true) {
    // Controller's output, contains status flag.
    robot_controller_.GetPrimitiveOutput(&output);
    if (output.status == drake::jjz::PrimitiveOutput::DONE)
    	break;
	}
	return;
}

bool RobotComm::CheckWhetherControlDone() {
	drake::jjz::PrimitiveOutput output;
  robot_controller_.GetPrimitiveOutput(&output);
  bool flag_done = (output.status == drake::jjz::PrimitiveOutput::DONE);
  return flag_done;
}
