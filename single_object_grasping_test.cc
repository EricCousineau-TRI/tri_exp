#include "robot_comm.h"
#include "perception.h"
#include "openni_comm.h"

const Eigen::Isometry3d tf_hand_to_ee(
    Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.2384)) *
    Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY()));

const std::string kPath =
    "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";



int main() {
	RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);
  double width_book = 0.18;
  double height_book = 0.23;
  double half_finger_thickness = 0.012;
	RigidBodyFrame<double> hand_frame("tool", tree.FindBody(drake::jjz::kEEName),
  		tf_hand_to_ee);

	RobotComm robot_comm(tree, hand_frame);
  OpenNiComm camera_interface;
	PointCloudPerception<ColoredPointT, ColoredPointTNormal> perception_proc;

	Eigen::Isometry3d tf_camera_wrt_ee;
  // tf_camera_wrt_ee.matrix() << 
  // 0.3994,   -0.9168,   -0.0015,   -0.0446,
  // 0.9163,    0.3992,   -0.0317,    0.0011,
  // 0.0297,    0.0113,    0.9995,    0.1121,
  //      0,         0,         0,    1.0000;
  tf_camera_wrt_ee.matrix() << 
    0.3994,   -0.9168,   -0.0015,   -0.0646,
    0.9163,    0.3992,   -0.0317,    0.00111,
    0.0297,    0.0113,    0.9995,    0.1121,
         0,         0,         0,    1.0000;

  Eigen::Isometry3d tf_camera_wrt_hand = tf_hand_to_ee.inverse() * tf_camera_wrt_ee;

  while (true) {
	 	Eigen::VectorXd joint_target_up(7), joint_target_left(7), joint_target_right(7);
	  joint_target_up << -49.9, -35.1 , 4.9, -86.3, 22.6, 113.0, 6.1;
	  joint_target_left << -106.767, -44.7901, 141.398, -36.7648, 0.0580792, 115.004, 106.54;
	  joint_target_right << -64.4928, 45.6072, 43.5198, -30.5794, 0.0839685, 114.999, -27.0017;
	  //joint_target_up << 13.7487,-26.2073,-3.65378,-90.8374,-5.463,92.5318,31.2576;
	  std::vector<Eigen::VectorXd> joint_targets;
	  //joint_targets.push_back(joint_target_right);
	  joint_targets.push_back(joint_target_up);
	  //joint_targets.push_back(joint_target_left);

	  double duration = 2.0;
  	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(
				new pcl::PointCloud<ColoredPointT>);
	  robot_comm.OpenGripper();
	  std::cin.get();

	  for (int i = 0; i < joint_targets.size(); ++i) {
	  	robot_comm.MoveToJointPositionDegrees(joint_targets[i], duration);
	  	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> single_cloud(
				new pcl::PointCloud<ColoredPointT>);
			camera_interface.GetCurrentPointCloud(single_cloud);

	 		Eigen::Isometry3d camera_pose_iso = robot_comm.GetCartesianPose() * tf_camera_wrt_hand;
			Eigen::Affine3f camera_pose;
			camera_pose.matrix() = camera_pose_iso.matrix().cast<float>();
			perception_proc.ApplyTransformToPointCloud(camera_pose, single_cloud);
			*cloud += *single_cloud;
	  }
		perception_proc.OutlierRemoval(cloud);		
		Eigen::Vector3f min_range;
		min_range << 0.4, -0.5, -0.02;
		Eigen::Vector3f max_range;
		max_range << 0.95, 0.5, 0.3;
		perception_proc.CutWithWorkSpaceConstraints(cloud, min_range, max_range);
		// Get rid of the table.
	  double thickness = 0.01;
	  perception_proc.SubtractTable(cloud, thickness);
	  pcl::io::savePCDFileASCII("test_grasping.pcd", *cloud);

		Eigen::Vector3f center, top_corner, lower_corner;
		Eigen::Matrix3f orientation;
		double cover_ratio = 0.85;
	  perception_proc.FindBoundingBox(cloud, &center, &top_corner, 
	  	&lower_corner, &orientation, cover_ratio);
	  double yaw_angle_radian = atan2(orientation(1,0), orientation(0,0));
	  std::cout << "Center point bbox: " << center.transpose() << std::endl;
	  std::cout << "Top corner: " << top_corner.transpose() << std::endl;
	  std::cout << "Lower corner: " << lower_corner.transpose() << std::endl;
	  std::cout << "Orientation: " << orientation << std::endl;
	  std::cout << "Yaw angle: " << yaw_angle_radian * 180 / M_PI << std::endl;

	  Eigen::VectorXd joint_buffer_above(7);
	  joint_buffer_above << 1.75373,11.7494,-2.09317,-86.2383,0.42104,82.5027,21.6225;
	  robot_comm.MoveToJointPositionDegrees(joint_buffer_above, duration);

	  double box_height = top_corner(2) - lower_corner(2);
	  double z_above = 0.2;
	  double z_grasp_height = 0;
	  if (top_corner(2) < 0.06) {
	  	z_grasp_height = 0;
	  } else {
	  	z_grasp_height = center(2) + box_height / 4.0;
	  }
	  Eigen::Isometry3d pose_approach = Eigen::Isometry3d::Identity();
	  pose_approach.linear() = pose_approach.linear() 
	  	* Eigen::AngleAxis<double>(yaw_angle_radian, Eigen::Vector3d::UnitZ());
	  pose_approach.translation() = center.cast<double>();
	  //pose_approach.translation()(2) += z_above; 
	  pose_approach.translation()(2) = z_grasp_height + z_above;
	  robot_comm.MoveToCartesianPose(pose_approach, 2.0);
	  pose_approach.translation()(2) = z_grasp_height;
	  robot_comm.MoveToCartesianPose(pose_approach, 2.0);
	  robot_comm.CloseGripper();
	  pose_approach.translation()(2) += 0.15;
	  robot_comm.MoveToCartesianPose(pose_approach, 2.0);

	  Eigen::VectorXd joint_place_above(7);
	  joint_place_above << -75.227,33.2758,-10.081,-94.8813,6.93116,52.7514,-65.92;
	  robot_comm.MoveToJointPositionDegrees(joint_place_above, 3.0);
	  robot_comm.OpenGripper();
	}
  
  return 0;
}