#include "efusion_interface.h"
#include "robot_comm.h"
#include "perception.h"
#include "anti_podal_grasp.h"
#include <string>
const std::string kPath =
    "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

const Eigen::Isometry3d tf_hand_to_ee(
    Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.18)) *
    Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY()));

int main(int argc, char** argv) {

	// pangolin::Params windowParams;
 //  windowParams.Set("SAMPLE_BUFFERS", 0);
 //  windowParams.Set("SAMPLES", 0);
 //  pangolin::CreateWindowAndBind("Main", 1080, 980, windowParams);

	RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);

	Eigen::Isometry3d tf_camera_wrt_ee;
  tf_camera_wrt_ee.matrix() << 
    0.3994,   -0.9168,   -0.0015,   -0.0646,
    0.9163,    0.3992,   -0.0317,    0.00111,
    0.0297,    0.0113,    0.9995,    0.1121,
         0,         0,         0,    1.0000;

  Eigen::Isometry3d tf_camera_wrt_hand = tf_hand_to_ee.inverse() * tf_camera_wrt_ee;

	RigidBodyFrame<double> eye_frame("tool", tree.FindBody(drake::jjz::kEEName),
  		tf_camera_wrt_ee);

  RigidBodyFrame<double> tool_frame("tool", tree.FindBody(drake::jjz::kEEName),
  		drake::jjz::X_ET);

	RobotComm robot_comm(tree, eye_frame);
	robot_comm.OpenGripper();
	PointCloudPerception<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> perception_proc;


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
	// perception_proc.OutlierRemoval(cloud);		
	Eigen::Vector3f min_range;
	min_range << 0.4, -0.5, -0.02;
	Eigen::Vector3f max_range;
	max_range << 0.95, 0.5, 0.3;
	
	int counter = 0;
	int num_original_size = cloud->size();
	std::cout << "Original size before ws cut " << num_original_size << std::endl;
	for (int i = 0; i < num_original_size; ++i) {
		if ((cloud->points[i].x >= min_range(0)) && (cloud->points[i].x <= max_range(0))
			&& (cloud->points[i].y >= min_range(1)) && (cloud->points[i].y <= max_range(1))
		  && (cloud->points[i].z >= min_range(2)) && (cloud->points[i].z <= max_range(2))) {
			cloud->points[counter] = cloud->points[i];
			counter++;
		}
	}
	cloud->resize(counter);
	std::cout << "After cut " << cloud->size() << std::endl;

	double dist_threshold = 0.01;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(dist_threshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	for (int i = 0; i < coefficients->values.size(); ++i) {
		std::cout << coefficients->values[i] << ",";
	}
	Eigen::Affine3f tf_shift = Eigen::Affine3f::Identity();
	tf_shift.matrix()(2,3) = coefficients->values[3];
	pcl::transformPointCloud(*cloud, *cloud, tf_shift);
	cin.get();
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud);


	std::string file_name_save = "efused_test.pcd";
	if (argc > 2) {
		file_name_save = std::string(argv[2]);
	}

	pcl::io::savePCDFileASCII(file_name_save, *cloud);
	std::cout << "saved " << std::endl;
	
	std::string config_file = "/home/user/gpg/cfg/params.cfg";
	std::cout << config_file << std::endl;
	
	AntiPodalGraspPlanner grasp_planner(config_file);
  grasp_planner.SetInputCloud(cloud);
  std::vector<AntiPodalGrasp> all_grasps = grasp_planner.GenerateAntipodalGrasp();


  if (all_grasps.size() >= 1) {
	  Eigen::VectorXd joint_buffer_above(7);
	  double duration_move_above = 2.0;
	  joint_buffer_above << 1.75373,11.7494,-2.09317,-86.2383,0.42104,82.5027,21.6225;
	  robot_comm.MoveToJointPositionDegrees(joint_buffer_above, duration_move_above);
	  Eigen::Isometry3d grasp_pose = all_grasps[0].hand_pose * tf_camera_wrt_hand;
	  std::cout << grasp_pose.matrix();
	  cin.get();
	  double duration_move_pregrasp = 5.0;
	  double z_above = 0.1;
	  grasp_pose.translation()(2) += z_above;
	  robot_comm.MoveToCartesianPose(grasp_pose, duration_move_pregrasp);
	  double duration_move_grasp = 2.0;
	  grasp_pose.translation()(2) -= z_above;
	  robot_comm.MoveToCartesianPose(grasp_pose, duration_move_grasp);
	  robot_comm.CloseGripper();
	}

	while(true) {};
	return 0;
}

