#include "efusion_interface.h"
#include "robot_comm.h"
#include "perception.h"
#include "anti_podal_grasp.h"
#include <string>
const std::string kPath =
    "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";


const Eigen::Isometry3d tf_hand_to_ee(
    Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.20)) *
    Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY()));

void InitializeOpenGL() {
	// pangolin::Params windowParams;
 //  windowParams.Set("SAMPLE_BUFFERS", 0);
 //  windowParams.Set("SAMPLES", 0);
 //  //pangolin::CreateWindowAndBind("Main", 1200, 980, windowParams);
 //  pangolin::CreateWindowAndBind("Main", 600, 480, windowParams);
	pangolin::CreateWindowAndBind("Main", 600, 480);
	//cv::namedWindow("test rgb");
}

void ExecuteGrasp(RobotComm& robot_comm, Eigen::Isometry3d& grasp_pose, 
		OpenNiComm& camera_interface) {
 	Eigen::VectorXd joint_buffer_above(7);
  double duration_move_above = 2.0;
  joint_buffer_above << 1.75373,11.7494,-2.09317,-86.2383,0.42104,82.5027,21.6225;
  robot_comm.MoveToJointPositionDegrees(joint_buffer_above, duration_move_above);
  std::cout << "Picked grasp pose(hand frame) " << std::endl;
  std::cout << grasp_pose.matrix() << std::endl;

  double duration_move_pregrasp = 2.5;
  double z_above = 0.1;
  grasp_pose.translation()(2) += z_above;

  robot_comm.MoveToCartesianPose(grasp_pose, duration_move_pregrasp);
  bool flag_display = true;
  cv::Mat rgb_img = camera_interface.GetCurrentRGBImage(flag_display);

  double duration_move_grasp = 2.0;
  grasp_pose.translation()(2) -= z_above;
  robot_comm.MoveToCartesianPose(grasp_pose, duration_move_grasp);
  // Grasp it. 
  robot_comm.CloseGripper();	
  // Wait for a bit.
  usleep(0.5 * 1e+6);
}

AntiPodalGrasp PickBestGrasp(std::vector<AntiPodalGrasp> all_grasps) {
	assert(all_grasps.size() >= 1);
	int rand_max_N = std::min(int(all_grasps.size()), 10);
	int grasp_to_choose_id = rand() % rand_max_N;
	return all_grasps[grasp_to_choose_id];
}

bool CheckReachableConstraint(Eigen::Isometry3d pose) {
	const double max_x = 0.725;
	const double min_x = 0.40;
	double pose_x = pose.translation()(0);
	if (pose_x > min_x && pose_x < max_x) {
		std::cout << "The grasp is reachable" << std::endl;
		return true;
	} else {
		std::cout << "The grasp is NOT reachable" << std::endl;
		return false;
	}
}

int main(int argc, char** argv) {

	// pangolin::Params windowParams;
 //  windowParams.Set("SAMPLE_BUFFERS", 0);
 //  windowParams.Set("SAMPLES", 0);
 //  pangolin::CreateWindowAndBind("Main", 1080, 980, windowParams);
	InitializeOpenGL();
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

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> cloud(new 
	pcl::PointCloud<pcl::PointXYZRGBNormal>);
	OpenNiComm camera_interface;
	
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

	while (true) {
		cloud->clear();
		EFusionInterface efusion_test;
		robot_comm.MoveToJointPositionDegrees(joint_targets[0], 2.5);

		for (int i = 1; i < joint_targets.size(); ++i) {
			bool flag_blocking = false;
			robot_comm.MoveToJointPositionDegrees(joint_targets[i], duration, flag_blocking);
			while (!robot_comm.CheckWhetherControlDone()) {
				Eigen::Isometry3d camera_pose = robot_comm.GetCartesianPose();
				//Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
				efusion_test.ProcFrame(camera_pose, camera_interface);
			}
		}
		
		efusion_test.GetFusedPointCloud(cloud);
		pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
		double leaf_size = 0.0005;
		grid.setLeafSize(leaf_size, leaf_size, leaf_size);
		grid.setInputCloud(cloud);
		grid.filter(*cloud);
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
		//cin.get();
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
	  // 5mm for the rubber wrap.
	  double minimum_finger_grasp_dist = (5.0 + 2.5) * 1e-3;
	  bool flag_grasp_success = false;
	  if (all_grasps.size() >= 1) {
	  	int trial_count = 0;
	  	do {
		    AntiPodalGrasp best_grasp = PickBestGrasp(all_grasps);
		  	Eigen::Isometry3d grasp_pose = best_grasp.hand_pose * tf_camera_wrt_hand;
		  	flag_grasp_success = false;
		  	bool flag_reachable = CheckReachableConstraint(grasp_pose);
		 		if (flag_reachable) {
		 			ExecuteGrasp(robot_comm, grasp_pose, camera_interface);
		 			std::cout << robot_comm.GetGripperFingerDistance() << std::endl;
			 		flag_grasp_success = 
		 				robot_comm.GetGripperFingerDistance() > minimum_finger_grasp_dist;
		 			if (!flag_grasp_success) {
		 				robot_comm.OpenGripper();
		 			}
		 		}
		 		trial_count++;
	  	} while ((!flag_grasp_success) && trial_count <= 5);
		  if (flag_grasp_success) {
			  // Lift up. 
			  double duration_lift_up = 2.0;
			  Eigen::Isometry3d cur_pose = robot_comm.GetCartesianPose();
			  cur_pose.translation()(2) += 0.3;
			  robot_comm.MoveToCartesianPose(cur_pose, duration_lift_up);
			  // Transport to bin.
			  Eigen::VectorXd joint_place_above(7);
			  //joint_place_above << -75.227,33.2758,-10.081,-94.8813,6.93116,52.7514,-65.92;
			  joint_place_above << -72.883,29.6285,-13.3159,-92.5593,7.56961,58.9519,-66.4744;
			  robot_comm.MoveToJointPositionDegrees(joint_place_above, 2.0);
			}
		  robot_comm.OpenGripper();
		}
	}


	return 0;
}

