#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <map>
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
#include "openni_comm.h"
#include "perception.h"

const Eigen::Isometry3d tf_hand_to_ee(
    Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.2384)) *
    Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY()));

const std::string kPath =
    "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    //split(s, delim, std::back_inserter(elems));
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
    	//std::cout << item << ",";
    	bool is_empty_space = (item.compare(std::string(" ")) == 0 ) || item.length() == 0;
    	  if (!is_empty_space) {
        	elems.push_back(item);
    	  }
    }
    return elems;
}

enum StringValue { evNotDefined, evSetTool, evMoveJ, evMoveC, evGetJ,
												  evGetC, evGuardedMove, evClose, evOpen };

std::map<std::string, StringValue> s_mapStringValues;

void InitializeKeyBoardMapping()
{
  s_mapStringValues["SetTool"] = evSetTool;
  s_mapStringValues["MoveJ"] = evMoveJ;
  s_mapStringValues["MoveC"] = evMoveC;
  s_mapStringValues["GetJ"] = evGetJ;
 	s_mapStringValues["GetC"] = evGetC;
 	s_mapStringValues["GuardedMove"] = evGuardedMove;
  s_mapStringValues["Close"] = evClose;
  s_mapStringValues["Open"] = evOpen; 
}

// Command line control of robot for rapid prototyping of robot experiments.
class CmdRobot {
 public:
	CmdRobot(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& tool_frame)
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

	  // Todo(Jiaji) : This is a hack just to use the existing api.
	  //std::cout << cur_pose_ee.matrix() << "\n\n";
	  //std::cout << tgt_pose_ee.matrix() << "\n\n";

	  //getchar();

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

	// "Close". Close the gripper.
	void CloseGripper();

	// "Open". Open the gripper.
	void OpenGripper();

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

void Run() {
	std::cout << "Robot control cmd line tool" << std::endl;
 	RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);
	RigidBodyFrame<double> tool_frame("tool", tree.FindBody(drake::jjz::kEEName),
  		tf_hand_to_ee);

	Eigen::Isometry3d tf_camera_wrt_ee;
  tf_camera_wrt_ee.matrix() << 
    0.3994,   -0.9168,   -0.0015,   -0.0446,
    0.9163,    0.3992,   -0.0317,    0.0011,
    0.0297,    0.0113,    0.9995,    0.1121,
         0,         0,         0,    1.0000;
  //Eigen::Isometry3d tf_camera_wrt_hand = drake::jjz::X_ET.inverse() * tf_camera_wrt_ee;
  RigidBodyFrame<double> camera_frame("camera", tree.FindBody(drake::jjz::kEEName),
                                      tf_camera_wrt_ee);

	double gaze_dist = 0.8;
 	Eigen::VectorXd q0 = Eigen::VectorXd::Zero(7);
  q0[1] = -11;
  q0[3] = -69;
  q0[5] = 109;
  q0 = q0 / 180. * M_PI;
		std::vector<Eigen::VectorXd> joint_targets =
    	drake::jjz::ComputeCalibrationConfigurations(
  	  		tree, camera_frame, q0, Eigen::Vector3d(0.65, 0, -0.1), 
    			gaze_dist, 1.5, 0.0, 2, 2);
  for (int i = 0; i < joint_targets.size(); ++i) {
  	std::cout << (180 / M_PI) * joint_targets[i].transpose() << std::endl;
  }

	//CmdRobot cmd_robot(tree, camera_frame);
	CmdRobot cmd_robot(tree, tool_frame);
	InitializeKeyBoardMapping();
	//OpenNiComm camera_interface;
	Eigen::Affine3f tf;
	Eigen::Isometry3d tf2;	
	PointCloudPerception<ColoredPointT, ColoredPointTNormal> test;
	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(
			new pcl::PointCloud<ColoredPointT>);
	Eigen::Vector4d coeffs_plane;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	double dist_threshold = 0.01;

	while (true) {
		std::string cmd_string;
		std::getline(std::cin, cmd_string);
		std::vector<std::string> elems = split(cmd_string, ' ');
		Eigen::VectorXd q_cmd(7);
		// xyzrpy pose.
		Eigen::VectorXd pose_cmd(6);		
		Eigen::Isometry3d tgt_pose = Eigen::Isometry3d::Identity();
		double fz;
		double mu;
		switch (s_mapStringValues[elems[0]]) {
			case evGetJ:
				cmd_robot.GetJointPosition();
				break;
			
			case evGetC:
				tf2 = cmd_robot.GetCartesianPosition();				
				// camera_interface.GetCurrentPointCloud(cloud);				
				// tf.matrix() = tf2.matrix().cast<float>();
				// test.ApplyTransformToPointCloud(tf, cloud);
				// test.VisualizePointCloudDrake(cloud);
				// test.FindPlane(cloud, &coeffs_plane, inliers, dist_threshold);
				// std::cout << coeffs_plane << std::endl;
				break;
			
			case evMoveJ:
				if (elems.size() == 9) {
					for (int i = 1; i < elems.size() - 1; ++i) {
						q_cmd[i - 1] = atof(elems[i].c_str());
					}
					double duration = atof(elems[elems.size() - 1].c_str());
					//std::cout << q_cmd.transpose() << std::endl;
					cmd_robot.MoveToJointPosition(q_cmd, duration);
				} else {
					std::cout << "wrong number of arguments. 7 dof + time" << std::endl;
				}
				break;

			case evMoveC:
				if (elems.size() >= 8 && elems.size () <= 10) {
					for (int i = 1; i < 7; ++i) {
						pose_cmd[i - 1] = atof(elems[i].c_str());
					}
					double duration = atof(elems[7].c_str());
					fz = 0;
					mu = 0;
					if (elems.size() >= 9) {
						fz = atof(elems[8].c_str());
					}
					if (elems.size() >= 10) {
						mu = atof(elems[9].c_str());
					}
					// Roll Pitch Yaw.
					Eigen::Matrix3f M_rot = 
						(Eigen::AngleAxisf(pose_cmd(3) / 180.0 * M_PI, Eigen::Vector3f::UnitX())
			    	* Eigen::AngleAxisf(pose_cmd(4) / 180.0 * M_PI, Eigen::Vector3f::UnitY())
						* Eigen::AngleAxisf(pose_cmd(5) / 180.0 * M_PI, Eigen::Vector3f::UnitZ())).toRotationMatrix();
					std::cout << M_rot << std::endl;
					tgt_pose.linear() = M_rot.cast<double>();
					std::cout << tgt_pose.matrix() << std::endl;
					tgt_pose.translation() = pose_cmd.head(3);
					cmd_robot.MoveToCartesianPosition(tgt_pose, duration, fz, mu);

				} else {
					std::cout << "wrong number of arguments. xyzrpy + time + fz(optional) + mu(optional)" 
					<< std::endl;
				}
				break;

			default:
				std::cout << "illegal input command type" << std::endl;
				break;
		}
	}
}


int main(int argc, char** argv) {

	Run();
	return 0;
}