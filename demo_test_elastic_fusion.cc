#include "efusion_interface.h"
#include "robot_comm.h"
#include "perception.h"

const std::string kPath =
    "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

int main(int argc, char** argv) {
	RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);

	Eigen::Isometry3d tf_camera_wrt_ee;
  tf_camera_wrt_ee.matrix() << 
    0.3994,   -0.9168,   -0.0015,   -0.0646,
    0.9163,    0.3992,   -0.0317,    0.00111,
    0.0297,    0.0113,    0.9995,    0.1121,
         0,         0,         0,    1.0000;


	RigidBodyFrame<double> eye_frame("tool", tree.FindBody(drake::jjz::kEEName),
  		tf_camera_wrt_ee);

	RobotComm robot_comm(tree, eye_frame);

	EFusionInterface efusion_test;
	
	Eigen::VectorXd joint_target_up(7), joint_target_left(7), joint_target_right(7);
	joint_target_up << -49.9, -35.1 , 4.9, -86.3, 22.6, 113.0, 6.1;
	joint_target_left << -106.767, -44.7901, 141.398, -36.7648, 0.0580792, 115.004, 106.54;
	joint_target_right << -64.4928, 45.6072, 43.5198, -30.5794, 0.0839685, 114.999, -27.0017;
	std::vector<Eigen::VectorXd> joint_targets;
	joint_targets.push_back(joint_target_right);
  joint_targets.push_back(joint_target_up);
	joint_targets.push_back(joint_target_left);

	double duration = 10.0;
	if (argc > 1) {
		duration = atof(argv[1]);
	}
	robot_comm.MoveToJointPositionDegrees(joint_targets[0], 2.5);
	for (int i = 1; i < joint_targets.size(); ++i) {
		bool flag_blocking = false;
		robot_comm.MoveToJointPositionDegrees(joint_targets[i], duration, flag_blocking);
		while (!robot_comm.CheckWhetherControlDone()) {
			Eigen::Isometry3d camera_pose = robot_comm.GetCartesianPose();
			//Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
			efusion_test.ProcFrame(camera_pose);
		}
	}

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> cloud(new 
		pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	efusion_test.GetFusedPointCloud(cloud);
	// pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
	// double leaf_size = 0.0005;
	// grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	// grid.setInputCloud(cloud);
	// grid.filter(*cloud);
	std::string file_name_save(argv[2]);
	pcl::io::savePCDFileASCII(file_name_save, *cloud);
	std::cout << "saved " << std::endl;
	while(true) {};
	return 0;
}

