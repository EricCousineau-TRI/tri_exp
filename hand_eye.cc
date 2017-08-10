#include "hand_eye.h"

HandEye::HandEye(const RigidBodyTree<double>& tree, const RigidBodyFrame<double>& tool_frame)
  : robot_controller_(tree, tool_frame),
  	robot_state_(&tree, &tool_frame) {
  robot_controller_.Start();
  usleep(1e+6);
}

void HandEye::SetEyeTransformationWrtHandFrame(
		const Eigen::Isometry3d& tf_eye_wrt_hand) {
	tf_eye_wrt_hand_ = tf_eye_wrt_hand;
}

// Get the pose of the camera with respect to robot base.
Eigen::Affine3f HandEye::GetCameraPoseWrtRobotBase() {
	UpdateRobotState();
	Eigen::Isometry3d pose_hand = robot_state_.get_X_WT();
	Eigen::Isometry3d pose_camera_to_convert = pose_hand * tf_eye_wrt_hand_;
  Eigen::Affine3f pose_camera;
  //std::cout << pose_camera_to_convert.matrix() << std::endl;
  pose_camera.matrix() = pose_camera_to_convert.matrix().cast<float>();
  // for (int i = 0; i < 3; ++i)
  // 	for (int j = 0; j <3; ++j)
  // 		pose_camera.matrix()(i,j) = (pose_camera_to_convert.matrix()(i,j));
  return pose_camera;
}

void HandEye::MoveCameraToCartesianPose(const Eigen::Isometry3d& tgt_pose_camera,
																				double duration) {
	Eigen::Isometry3d tgt_pose_hand = tgt_pose_camera * tf_eye_wrt_hand_.inverse();
	MoveEECartesianStraightLine(tgt_pose_hand, duration);
}

void HandEye::MoveHandToCartesianPose(const Eigen::Isometry3d& tgt_pose_ee,
																			double duration) {
	MoveEECartesianStraightLine(tgt_pose_ee, duration);
}

void HandEye::WaitUntilControlAckDone() {
 	drake::jjz::PrimitiveOutput output;
	while(true) {
    // Controller's output, contains status flag.
    robot_controller_.GetPrimitiveOutput(&output);
    if (output.status == drake::jjz::PrimitiveOutput::DONE)
    	break;
	}
	return;
}

void HandEye::UpdateRobotState() {
	// The robot controller gets the current robot state from LCM message
	// and update the state. Kinematics cache is updated.
	robot_controller_.GetState(&robot_state_);
}

void HandEye::MoveEECartesianStraightLine(const Eigen::Isometry3d& tgt_pose_ee,
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

  std::cout << cur_pose_ee.matrix() << "\n\n";
  std::cout << tgt_pose_ee.matrix() << "\n\n";

  getchar();

	robot_controller_.MoveToolFollowTraj(traj, dummy_fz, dummy_mu, dummy_mu);
	WaitUntilControlAckDone();
}

void HandEye::MoveRobotJointStraightLine(const Eigen::VectorXd& tgt_joint, 
		double duration) {
	robot_controller_.MoveJ(tgt_joint, duration);
	WaitUntilControlAckDone();
}

void HandEye::Scan(OpenNiComm& camera_interface, 
		PointCloudPerception<ColoredPointT, ColoredPointTNormal> & perception_proc, 
		const std::vector<Eigen::VectorXd>& joint_angles, 
		double duration_per_move, 
		boost::shared_ptr<pcl::PointCloud<ColoredPointTNormal>> fused_cloud) {
	
	int num_moves = joint_angles.size();
	std::vector< boost::shared_ptr<pcl::PointCloud<ColoredPointT>> > point_clouds;
	Eigen::Affine3f first_camera_pose;
	for (int i = 0; i < num_moves; ++i) {
		MoveRobotJointStraightLine(joint_angles[i], duration_per_move);
		std::cout << "Moved to target position" << std::endl;
		boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(new pcl::PointCloud<ColoredPointT>);  
		usleep(0.5*1e+6);
		camera_interface.GetCurrentPointCloud(cloud);
		//std::cout << cloud->size() << std::endl;
		//perception_proc.VisualizePointCloud(cloud);
		cin.get();
		const Eigen::Affine3f camera_pose = GetCameraPoseWrtRobotBase();
		std::cout << camera_pose.matrix() << std::endl;
		if (i == 0) {
			first_camera_pose = camera_pose;
		}
		perception_proc.ApplyTransformToPointCloud(camera_pose, cloud);
	  std::vector<int> indices;
	  pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
	  perception_proc.OutlierRemoval(cloud);
	  //perception_proc.DownSample(cloud, 0.00001);
		Eigen::Vector3f min_range;

		min_range << 0,-0.4,-0.2;
		Eigen::Vector3f max_range;
		max_range << 0.8,0.4,0.5;
		perception_proc.CutWithWorkSpaceConstraints(cloud, min_range, max_range);
		point_clouds.push_back(cloud);
	}
	std::cout << "to fuse " << point_clouds.size() << " clouds" << std::endl;
	perception_proc.FuseMultiPointClouds(point_clouds, fused_cloud);
	std::cout << "fused cloud size " << fused_cloud->size() << std::endl;
	// boost::shared_ptr<pcl::PointCloud<ColoredPointT>> vis_pointcloud;
	// boost::shared_ptr<pcl::PointCloud<pcl::Normal>> vis_normal;
	// pcl::copyPointCloud(*fused_cloud, *vis_pointcloud);
	// pcl::copyPointCloud(*fused_cloud, *vis_normal);
	//perception_proc.VisualizePointCloudAndNormal(vis_pointcloud, vis_normal);
	
	//perception_proc.VisualizePointCloud(fused_cloud);
}


int main() {

  PointCloudPerception<ColoredPointT, ColoredPointTNormal> perception_proc;
     boost::shared_ptr<pcl::PointCloud<ColoredPointT>> tmp_cloud(new pcl::PointCloud<ColoredPointT>);
  perception_proc.LoadPCDFile("test_pcd.pcd", tmp_cloud);
  std::cout << "tmp cloud size" << tmp_cloud->size() << std::endl;
  //perception_proc.VisualizePointCloud(tmp_cloud);

	const std::string kPath =
      "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf";

  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);

  RigidBodyFrame<double> tool_frame("tool", tree.FindBody(drake::jjz::kEEName),
  		drake::jjz::X_ET);

  // Our calbiration result last time.
  Eigen::Isometry3d tf_camera_wrt_ee;
  tf_camera_wrt_ee.matrix() <<
  	0.37451879, -0.92700796,  0.        , -0.06087369,
		0.92628617,  0.37501423,  0.        ,  0.03164403,
		0.04158859, -0.00453463,  1.        ,  0.105     ,
    0.        ,  0.        ,  0.        ,  1.0;
  Eigen::Isometry3d tf_camera_wrt_hand = drake::jjz::X_ET.inverse() * tf_camera_wrt_ee;
  HandEye hand_eye_system(tree, tool_frame);


  hand_eye_system.SetEyeTransformationWrtHandFrame(tf_camera_wrt_hand);
  Eigen::VectorXd joint_target1(7), joint_target2(7), joint_target3(7);
  joint_target1 << -65.23,45.69,-0.79,-49.51,44.84,114.14,-38.57;
  joint_target2 << -9.45,7.34,-0.79,-61.97,8.35,103.92,4.51;
  joint_target3 << 46.52,45.1 ,-0.78, -37.76, -36.19, 115.48, -31.81;
  joint_target1 = joint_target1 / 180 * M_PI;
  joint_target2 = joint_target2 / 180 * M_PI;
  joint_target3 = joint_target3 / 180 * M_PI;
  
  double duration_movement = 5;
  std::vector<Eigen::VectorXd> joint_targets;
  joint_targets.push_back(joint_target1);
  joint_targets.push_back(joint_target2);
  joint_targets.push_back(joint_target3);


  //camera_interface.Run();

	
	OpenNiComm camera_interface;

  boost::shared_ptr<pcl::PointCloud<ColoredPointTNormal>> fused_cloud (
  		new pcl::PointCloud<ColoredPointTNormal>);
  
  hand_eye_system.Scan(camera_interface, perception_proc, 
  		joint_targets, duration_movement, fused_cloud);
  pcl::io::savePCDFileASCII("test_fused.pcd", *fused_cloud);
  camera_interface.Stop();

  // hand_eye_system.MoveRobotJointStraightLine(joint_target1, duration_movement);
  // hand_eye_system.MoveRobotJointStraightLine(joint_target2, duration_movement);
  // hand_eye_system.MoveRobotJointStraightLine(joint_target3, duration_movement);
  
  // Eigen::Isometry3d pose_camera_in_world = hand_eye_system.GetCameraPoseWrtRobotBase();
  // std::cout << pose_camera_in_world.matrix() << std::endl;
  // pose_camera_in_world.translation()(2) += 10 * 1e-3;
  // hand_eye_system.MoveCameraToCartesianPose(pose_camera_in_world);
  // std::cout << "Move completed" << std::endl;
  
  while (true) {
  };

  return 0;
}
